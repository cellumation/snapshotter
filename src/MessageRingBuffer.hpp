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
#include "ShapeShifterMsg.hpp"
#include <deque>
#include <functional>
#include <mutex>
#include <ros/time.h>

namespace rosbag
{
class Bag;
}

namespace snapshotter
{
// TODO rename this to something more specific. It is a very specific class :D
/** A simple ring buffer to store ros messages*/
class MessageRingBuffer
{
public:
    /** @param maxSize in bytes
     *  @param droppedCb a callback that is invoked every time a message is
     *                   dropped from this buffer.
     *                   signature: void cb(BufferEntry&& entry) */
    MessageRingBuffer(size_t maxSize, std::function<void(BufferEntry&&)> droppedCb) :
        maxSize(maxSize),
        droppedCb(droppedCb),
        currentSize(0)
    {}

    MessageRingBuffer(const MessageRingBuffer& other);

    /** Add a new message to the ring buffer.
     *  If there is no space left the oldest message will be deleted.
     *  is thread-safe */
    void push(ShapeShifterMsg::ConstPtr msg, const ros::Time& receiveTime);

    /** Writes the content of the buffer to a bag file.
     *  @throw BagWriteException in case of error
     *  is thread-safe */
    void writeToBag(rosbag::Bag& bag) const;

    /** Removes all entries from this buffer.
     *  is thread-safe */
    void clear();

    /** Returns the oldest received-timestamp currently present in the buffer.
     *  is thread-safe */
    ros::Time getOldestReceiveTime() const;

    void setDroppedCb(std::function<void(BufferEntry&&)> cb);

private:
    size_t maxSize; // in bytes //TODO rename to maxSizeBytes
    std::deque<BufferEntry> buffer;
    mutable std::mutex bufferLock;
    std::function<void(BufferEntry&&)> droppedCb;
    size_t currentSize; // in bytes
};

} // namespace snapshotter