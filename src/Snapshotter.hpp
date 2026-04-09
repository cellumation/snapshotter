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
#pragma once
#include "Common.hpp"
#include "MessageRingBuffer.hpp"
#include "SingleMessageBuffer.hpp"
#include <atomic>
#include <memory>
#include <rclcpp/node.hpp>
#include <shared_mutex>
#include <thread>
#include <unordered_set>
#include <vector>

namespace snapshotter
{
class Snapshotter
{
public:
    struct Config
    {
        size_t maxMemoryBytes;
        /** If true the last message of each latched topic will be kept inside the buffer.         */
        bool keepLatched;
        /** If true the snapshotter will set the thread nice value to 19 when
         *  writing a bag file. see manpage setpriority(2) for details. */
        bool niceOnWrite;
    };

    Snapshotter(rclcpp::Node& nh, const Config& cfg);

    /** The destructor will block until the currently running write operation has finished. */
    ~Snapshotter();

    /** The snapshotter will subscribe to the given @p topic and log it.
     *  If the topic is already subscribed nothing will happen. */
    bool subscribe(const std::string& topic);

    /** The snapshotter will subscribe to the service event topic for the given @p serviceName and log it.
     *  The service name should be the plain service name (e.g. /my_service), not the event topic name.
     *  If the service event topic is already subscribed nothing will happen. */
    bool subscribeService(const std::string& serviceName);

    /** The callback will be invoked when the writing is either done or an error occurred.
     *  It will be invoked from a different thread than the one that called writeBagFile.
     *  @note The callback may not throw an exception.
     *  @p error will contain the error, if any error occurred. If writing finished successfully it will be nullopt.
     *  @p firstTimestamp will be the first timestamp in the bag (excluding latched topics).
     *  @p lastTimestamp will be the last timestamp in the bag.*/
    using WriteDoneCb = std::function<void(const std::optional<BagWriteException>& error,
                                           const rclcpp::Time& firstTimestamp, const rclcpp::Time& lastTimestamp)>;

    /** async writes the bag file.
     *  Recording of data continues while writing.
     *  This method is not blocking.
     *  @param cb will be invoked when the writing is either done or an error occurred.*/
    void writeBagFile(const std::string& path, BagCompression compression, const WriteDoneCb& cb);

private:
    void topicCB(const SerializedMsgPtr& msg, const TopicMetadata& md);

    /** is invoked every time a message is dropped from the MessageRingBuffer */
    void messageDroppedFromBufferCB(BufferEntry&& entry);

    rclcpp::Node& nh;
    Config cfg;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers;
    std::unordered_set<std::string> subscribedTopics;
    MessageRingBuffer buffer;
    SingleMessageBuffer lastDroppedLatchedMsgs;
    std::shared_mutex writeBagLock;

    std::mutex metaDataLock;
    std::vector<TopicMetadata> topicMetadata;

    rclcpp::Logger log;

    /** used to ensure that only one write operation is in progress at a time */
    std::mutex writerRunningLock;
    /** The thread used for all write operations */
    std::jthread writer;
};

} // namespace snapshotter
