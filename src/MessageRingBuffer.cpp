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
#include <rosbag/bag.h>

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

void MessageRingBuffer::push(ShapeShifterMsg::ConstPtr msg, const ros::Time& receiveTime)
{
    const size_t entrySize = msg->objectSize() + sizeof(ros::Time);

    if(entrySize > maxSize)
    {
        ROS_WARN_STREAM("Message from " << msg->getTopic() << " is too large for buffer. Ignoring message");
        return;
    }

    std::vector<BufferEntry> droppedMsgs;
    {
        std::scoped_lock lock(bufferLock);

        while(!buffer.empty() && currentSize + entrySize > maxSize)
        {
            //we accumulate all dropped messages to invoke the droppedCallback after unlocking.
            //The callback might take a long time and potentially lock other resources causing a deadlock
            currentSize -= buffer.front().msg->objectSize() + sizeof(ros::Time);
            droppedMsgs.emplace_back(std::move(buffer.front()));
            buffer.pop_front();
        }
        if(currentSize + entrySize <= maxSize)
        {
            currentSize += entrySize;
            buffer.emplace_back(std::move(msg), receiveTime);
        }
        else
        {
            ROS_ERROR_STREAM("Message from " << msg->getTopic() << " is too large for buffer");
        }
    }

    for(BufferEntry& e : droppedMsgs)
    {
        droppedCb(std::move(e));
    }
}

void MessageRingBuffer::writeToBag(rosbag::Bag& bag) const
{
    std::scoped_lock lock(bufferLock);
    for(const BufferEntry& entry : buffer)
    {
        bag.write(entry.msg->getTopic(), entry.receiveTime, entry.msg,
                entry.msg->getConnectionHeader());
    }
}

void MessageRingBuffer::clear()
{
    std::scoped_lock lock(bufferLock);
    while(!buffer.empty())
    {
        droppedCb(std::move(buffer.front()));
        buffer.pop_front();
    }
    currentSize = 0;
}

ros::Time MessageRingBuffer::getOldestReceiveTime() const
{
    std::scoped_lock lock(bufferLock);
    ros::Time oldestTime = ros::Time::now() + ros::Duration(10); //start with a timestamp in the future
    for(const BufferEntry& entry : buffer)
    {
        oldestTime = std::min(oldestTime, entry.receiveTime);
    }
    return oldestTime;
}

}