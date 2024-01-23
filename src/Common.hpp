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
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <stdexcept>

namespace snapshotter
{
using SerializedMsgPtr = std::shared_ptr<const rclcpp::SerializedMessage>;

static const rclcpp::Time minValidTimeStamp = rclcpp::Time(0, 0, RCL_ROS_TIME);

enum class BagCompression
{
    NONE,
    SLOW, // good compression but takes a long time
    FAST // a little bit of compression but fast
};

struct BagWriteException : std::runtime_error
{
    using std::runtime_error::runtime_error;
};

struct TopicMetadata
{
    TopicMetadata(const rosbag2_storage::TopicMetadata& rosMetadata, bool transientLocal, size_t metaDataIdx) :
        rosMetadata(rosMetadata),
        transientLocal(transientLocal),
        metaDataIdx(metaDataIdx)

    {}
    rosbag2_storage::TopicMetadata rosMetadata;
    bool transientLocal;

    // unique index / id of the metadata.
    size_t metaDataIdx;
};

struct BufferEntry
{
    /**ATTENTION The BufferEntries are copied when writing to the file.
     *           This becomes a problem when the BufferEntry grows in size.
     *           Currently this is fine because it only contains a pointer a timestamp, and
     *           and index to the associated metadata.
     *           DO NOT ADD MORE ATTRIBUTES! (or if you do, rethink how data is written
     *           to the bag file) */

    SerializedMsgPtr msg;
    rclcpp::Time receiveTime;

    // index of the metadata of the topic this message was received on
    uint16_t topicMetaDataIdx;

    BufferEntry(SerializedMsgPtr msg, const rclcpp::Time& receiveTime, uint16_t topicMetaDataIdx) :
        msg(std::move(msg)),
        receiveTime(receiveTime),
        topicMetaDataIdx(topicMetaDataIdx)
    {}

    BufferEntry(SerializedMsgPtr msg, const rclcpp::Time& receiveTime, const TopicMetadata& md) :
        msg(std::move(msg)),
        receiveTime(receiveTime),
        topicMetaDataIdx(md.metaDataIdx)
    {}
};

} // namespace snapshotter
