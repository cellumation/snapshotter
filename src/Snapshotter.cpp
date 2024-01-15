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
#include "Snapshotter.hpp"
#include <cerrno>
#include <rosbag2_compression/sequential_compression_writer.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sys/resource.h>
#include <thread>

namespace snapshotter
{
Snapshotter::Snapshotter(rclcpp::Node& nh, const Snapshotter::Config& cfg) :
    nh(nh),
    cfg(cfg),
    buffer(cfg.maxMemoryBytes, [this](BufferEntry&& e) { messageDroppedFromBufferCB(std::move(e)); }),
    log(nh.get_logger())
{}

bool Snapshotter::subscribe(const std::string& topic)
{
    if (subscribedTopics.find(topic) == subscribedTopics.end())
    {
        std::optional<TopicMetadata> md;

        std::vector<rclcpp::TopicEndpointInfo> pubInfo = nh.get_publishers_info_by_topic(topic);
        for (const rclcpp::TopicEndpointInfo& info : pubInfo)
        {
            bool transientLocal = info.qos_profile().durability() == rclcpp::DurabilityPolicy::TransientLocal;

            if (md)
            {
                if (md->topicTypeName != info.topic_type())
                {
                    RCLCPP_WARN_STREAM(log, "Error, node " << info.node_name()
                                                           << " has different message type than previous nodes "
                                                           << md->topicTypeName << " vs " << info.topic_type());
                    continue;
                }

                // if any publisher is transient local, we subscribe as transient local
                md->transient_local |= transientLocal;
            }

            md = TopicMetadata(topic, info.topic_type(), transientLocal, topicMetadata.size());
        };

        if (!md)
        {
            RCLCPP_ERROR_STREAM_ONCE(log, "Error, could not retreive endpoint information for topic " << topic);
            return false;
        }

        RCLCPP_INFO_STREAM(log, "Subscribing to: " << topic);

        rclcpp::QoS qos(50);
        if (md->transient_local)
        {
            qos.transient_local();
        }

        TopicMetadata finalMd = *md;
        {
            std::lock_guard l(metaDataLock);
            // push before subscription, the callback might access this
            topicMetadata.push_back(finalMd);
        }

        rclcpp::SubscriptionBase::SharedPtr sub = nh.create_generic_subscription(
            md->topicName, md->topicTypeName, qos, [this, finalMd](SerializedMsgPtr msg) { topicCB(msg, finalMd); });

        subscribers.push_back(sub);
        subscribedTopics.emplace(topic);
    }

    return true;
}

void Snapshotter::writeBagFile(const std::string& path, BagCompression compression)
{
    using namespace rosbag2_cpp;

    std::unique_ptr<rosbag2_cpp::writer_interfaces::BaseWriterInterface> writerImpl;

    switch (compression)
    {
        case BagCompression::FAST:
        {
            rosbag2_compression::CompressionOptions ops;
            ops.compression_mode = rosbag2_compression::CompressionMode::MESSAGE;
            ops.compression_format = "zstd";
            ops.compression_threads = 5;
            ops.compression_queue_size =
                0; // 0 means: never drop messages. If no free compression thread is available, wait
            writerImpl = std::make_unique<rosbag2_compression::SequentialCompressionWriter>(ops);
        }
        break;
        case BagCompression::SLOW:
        {
            rosbag2_compression::CompressionOptions ops;
            ops.compression_mode = rosbag2_compression::CompressionMode::FILE;
            ops.compression_format = "zstd";
            ops.compression_threads = 5;
            ops.compression_queue_size = 0; // as far as I understand this is not used in FILE compression mode
            writerImpl = std::make_unique<rosbag2_compression::SequentialCompressionWriter>(ops);
        }
        break;
        case BagCompression::NONE:
            writerImpl = std::make_unique<writers::SequentialWriter>();
            break;
        default:
            throw BagWriteException("Unhandled enum value");
    }

    rosbag2_cpp::Writer writer(std::move(writerImpl));

    std::unique_ptr<MessageRingBuffer> bufferCopy;
    std::unique_ptr<SingleMessageBuffer> latchedBufferCopy;
    {
        // wait until all threads have stopped writing to the buffers,
        // then copy both buffers
        std::unique_lock lock(writeBagLock);
        bufferCopy = std::make_unique<MessageRingBuffer>(buffer);
        latchedBufferCopy = std::make_unique<SingleMessageBuffer>(lastDroppedLatchedMsgs);
    }

    std::vector<TopicMetadata> metaData;
    {
        std::unique_lock l(metaDataLock);
        metaData = topicMetadata;
    }

    // replace the dropped-callback of the copied buffer. Otherwise dropped messages
    // from that buffer would end up in the original lastDroppedLatchedMsgs buffer.
    bufferCopy->setDroppedCb([](BufferEntry&&) {});

    // writing is done in a seperate thread because errors might happen
    // during writing and there is no guaranteed way to reset the
    // thread priority once an error occurred. If we would use one
    // of the ros threads we would leave a ros thread with very low
    // priority behind in case of error.
    // Even with perfect error handling the call to reset the
    // priority could still fail (and in fact did fail on my machine).
    // Thus it is much easier and safer to spawn a new thread and let
    // it die once we are done writing.
    std::exception_ptr ex = nullptr;
    std::thread t([&] {
        try
        {
            if (cfg.niceOnWrite)
            {
                // according to 'man setpriority' this call is not POSIX conform, and sets the priority
                // for this thread and all of its child threads. This exactly what we want.
                errno = 0;
                if (0 != setpriority(PRIO_PROCESS, 0, 19) || errno != 0)
                {
                    const std::string msg = "setpriority failed: " + std::string(std::strerror(errno));
                    RCLCPP_ERROR_STREAM(log, msg);
                    ex = std::make_exception_ptr(BagWriteException(msg));
                    return;
                }
            }

            writer.open(path);
            /** write all old latched messages 3 seconds before the actual log starts.
             *  The value 3 is arbitrary. The idea is to make the old latched messages stand out
             *  to a human reader when looking at the bag. We do the calculation in double because rclcpp::Time will
             * throw when the time becomes negative (which can happen when running in simulation because sim time starts
             * at 0) */
            rclcpp::Time latchedTime = bufferCopy->getOldestReceiveTime() - std::chrono::seconds(3);
            latchedTime = std::max(latchedTime, rclcpp::Time(static_cast<int64_t>(0), RCL_ROS_TIME));
            latchedBufferCopy->writeToBag(writer, metaData, latchedTime);

            bufferCopy->writeToBag(writer, metaData);
            writer.close();
        }
        catch (const std::exception& e)
        {
            ex = std::make_exception_ptr(BagWriteException(e.what()));
        }
        catch (...) // we really really don't want to crash :D
        {
            ex = std::make_exception_ptr(BagWriteException("unknown error"));
        }
    });
    t.join();

    if (ex)
    {
        std::rethrow_exception(ex);
    }
}

void Snapshotter::topicCB(const SerializedMsgPtr& msg, const TopicMetadata& md)
{
    // get time before we aquire the lock, might be more accurate
    rclcpp::Time curTime = nh.now();
    // multiple threads may work on the buffer at the same time
    // as long as no thread is trying to write the buffer to disk
    std::shared_lock lock(writeBagLock);

    buffer.push(msg, curTime, md);
}

void Snapshotter::messageDroppedFromBufferCB(BufferEntry&& entry)
{
    if (!cfg.keepLatched)
    {
        return;
    }
    /** we keep the last dropped message of each latched topic in a seperate buffer.
     *  Upon writing of the bag file the latched message is written a few seconds before
     *  the actual start of the bag file. The timestamp correction is only done for
     *  usability reasons (rqt_bag gui becomes unusable when we dont correct the
     *  timestamp of old latched messages).
     */

    bool isLatched = false;
    {
        std::lock_guard l(metaDataLock);
        isLatched = topicMetadata[entry.topicMetaDataIdx].transient_local;
    }

    if (isLatched)
    {
        /** In very rare cases newer messages might end up in the lastDroppedLatchedMsgs before older ones.
         *  To avoid overwriting them we keep the newer one inside the buffer.
         *
         *  Newer messages may arrive here before older ones for several reasons:
         *  (1) multiple messages of the same topic are dropped by seperate threads
         *      at nearly the same time from the buffer. Depending on the scheduler those messages
         *      may show up here in the wrong order.
         *  (2) Multiple messages of the same topic arrived in the topicCb at the same time. Depending
         *      on scheduling their order might have been inverted when pushing them to the buffer.
         *  (3) Multiple messages on the same topic originating from different publishers arrive in the
         *      wrong order due to network latency.
         *
         *  Handling all those cases correctly would require a far more complex design of the snapshotter.
         *  I am unwilling to increase design complexity to handle those rare corner cases.
         *  Instead we simply keep the newer of two entries in the lastDroppedLatchedMsgs. This should
         *  catch most of the cases (that where already very rare to begin with). This is good enough.
         **/
        lastDroppedLatchedMsgs.push(std::move(entry), true);
    }
}

} // namespace snapshotter
