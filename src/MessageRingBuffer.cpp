/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Cellumation GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************/
#include "MessageRingBuffer.hpp"
#include <rclcpp/logging.hpp>
#include <regex>
#include <rosbag2_cpp/writer.hpp>
#include <unordered_map>

namespace snapshotter
{
MessageRingBuffer::MessageRingBuffer(const MessageRingBuffer& other)
{
    std::scoped_lock lock(other.bufferLock);
    maxSize = other.maxSize;
    currentSize = other.currentSize;
    buffer = other.buffer;
    droppedCb = other.droppedCb;
}

void MessageRingBuffer::setDroppedCb(std::function<void(BufferEntry&&)> cb)
{
    droppedCb = cb;
}

static size_t getMemorySize(const SerializedMsgPtr& msg)
{
    return sizeof(rclcpp::SerializedMessage) + msg->capacity();
}

static size_t getMemorySize(const BufferEntry& e)
{
    return sizeof(BufferEntry) + getMemorySize(e.msg);
}

void MessageRingBuffer::push(SerializedMsgPtr msg, const rclcpp::Time& receiveTime, const TopicMetadata& md)
{
    BufferEntry e(std::move(msg), receiveTime, md);

    const size_t entrySize = getMemorySize(e);

    if (entrySize > maxSize)
    {
        rclcpp::Logger log = rclcpp::get_logger("snapshotter");
        RCLCPP_WARN_STREAM(log, "Message from " << md.rosMetadata.name << " is too large for buffer. Ignoring message");
        return;
    }

    std::vector<BufferEntry> droppedMsgs;
    {
        std::scoped_lock lock(bufferLock);

        while (!buffer.empty() && currentSize + entrySize > maxSize)
        {
            // we accumulate all dropped messages to invoke the droppedCallback after unlocking.
            // The callback might take a long time and potentially lock other resources causing a deadlock
            currentSize -= getMemorySize(buffer.front());
            droppedMsgs.emplace_back(std::move(buffer.front()));
            buffer.pop_front();
        }
        if (currentSize + entrySize <= maxSize)
        {
            currentSize += entrySize;
            buffer.push_back(std::move(e));
        }
        else
        {
            rclcpp::Logger log = rclcpp::get_logger("snapshotter");
            RCLCPP_ERROR_STREAM(log, "Message from " << md.rosMetadata.name << " is too large for buffer");
        }
    }

    for (BufferEntry& e : droppedMsgs)
    {
        droppedCb(std::move(e));
    }
}

void MessageRingBuffer::writeToBag(rosbag2_cpp::Writer& writer, const std::vector<TopicMetadata>& topicMetadata) const
{
    std::scoped_lock lock(bufferLock);

    for (const TopicMetadata& md : topicMetadata)
    {
        writer.create_topic(md.rosMetadata);
    }

    for (const BufferEntry& entry : buffer)
    {
        const TopicMetadata& md(topicMetadata[entry.topicMetaDataIdx]);
        if (entry.receiveTime >= minValidTimeStamp)
        {
            writer.write(entry.msg, md.rosMetadata.name, md.rosMetadata.type, entry.receiveTime);
        }
        else
        {
            // FIXME is this the workaround for the crash on time zero of rosbag1 ?
            writer.write(entry.msg, md.rosMetadata.name, md.rosMetadata.type, minValidTimeStamp);

            rclcpp::Logger log = rclcpp::get_logger("snapshotter");
            RCLCPP_WARN_STREAM(log,
                               "Message timestamp < rclcpp::Time(0). Replacing timestamp with rclcpp::Time(0). Topic:"
                                   << md.rosMetadata.name);
        }
    }
}

void MessageRingBuffer::writeToBag(rosbag2_cpp::Writer& writer, const std::vector<TopicMetadata>& topicMetadata,
                                   const std::vector<ReductionRule>& rules) const
{
    std::scoped_lock lock(bufferLock);

    for (const TopicMetadata& md : topicMetadata)
    {
        writer.create_topic(md.rosMetadata);
    }

    // Build a map from topicMetaDataIdx to the first matching rule (nullptr = no rule).
    // This avoids re-running every regexp for every message.
    std::unordered_map<uint16_t, const ReductionRule*> ruleForTopic;
    for (size_t i = 0; i < topicMetadata.size(); ++i)
    {
        const std::string& topicName = topicMetadata[i].rosMetadata.name;
        const ReductionRule* matched = nullptr;
        for (const ReductionRule& rule : rules)
        {
            const std::regex& topicRegexp =
                std::visit([](const auto& r) -> const std::regex& { return r.topicRegexp; }, rule);
            if (std::regex_match(topicName, topicRegexp))
            {
                matched = &rule;
                break;
            }
        }
        ruleForTopic[static_cast<uint16_t>(i)] = matched;
    }

    // Per-topic state for rate limiting.
    std::unordered_map<uint16_t, rclcpp::Time> lastWrittenTime;

    for (const BufferEntry& entry : buffer)
    {
        const ReductionRule* rule = ruleForTopic.at(entry.topicMetaDataIdx);
        if (rule)
        {
            const bool shouldWrite = std::visit(
                [&](const auto& r) -> bool {
                    using T = std::decay_t<decltype(r)>;
                    if constexpr (std::is_same_v<T, DropRule>)
                    {
                        return false; // drop this message
                    }
                    else if constexpr (std::is_same_v<T, ReduceRule>)
                    {
                        auto it = lastWrittenTime.find(entry.topicMetaDataIdx);
                        if (it == lastWrittenTime.end())
                        {
                            lastWrittenTime.emplace(entry.topicMetaDataIdx, entry.receiveTime);
                            return true;
                        }
                        else
                        {
                            const rclcpp::Duration elapsed = entry.receiveTime - it->second;
                            if (elapsed < r.minInterval)
                            {
                                return false; // skip this message
                            }
                            it->second = entry.receiveTime;
                            return true;
                        }
                    }
                },
                *rule);

            if (!shouldWrite)
            {
                continue;
            }
        }

        const TopicMetadata& md(topicMetadata[entry.topicMetaDataIdx]);
        const rclcpp::Time writeTime = entry.receiveTime >= minValidTimeStamp ? entry.receiveTime : minValidTimeStamp;
        writer.write(entry.msg, md.rosMetadata.name, md.rosMetadata.type, writeTime);
    }
}

void MessageRingBuffer::clear()
{
    std::scoped_lock lock(bufferLock);
    while (!buffer.empty())
    {
        droppedCb(std::move(buffer.front()));
        buffer.pop_front();
    }
    currentSize = 0;
}

rclcpp::Time MessageRingBuffer::getOldestReceiveTime() const
{
    std::scoped_lock lock(bufferLock);
    if (buffer.empty())
    {
        return minValidTimeStamp;
    }

    rclcpp::Time oldestTime = buffer.front().receiveTime;
    for (const BufferEntry& entry : buffer)
    {
        oldestTime = std::min(oldestTime, entry.receiveTime);
    }
    return oldestTime;
}

rclcpp::Time MessageRingBuffer::getNewestReceiveTime() const
{
    std::scoped_lock lock(bufferLock);
    if (buffer.empty())
    {
        return minValidTimeStamp;
    }

    rclcpp::Time newestTime = buffer.front().receiveTime;
    for (const BufferEntry& entry : buffer)
    {
        newestTime = std::max(newestTime, entry.receiveTime);
    }
    return newestTime;
}
} // namespace snapshotter
