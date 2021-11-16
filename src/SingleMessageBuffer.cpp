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
#include "SingleMessageBuffer.hpp"
#include <rosbag/bag.h>

namespace snapshotter
{

SingleMessageBuffer::SingleMessageBuffer(const SingleMessageBuffer& other)
{
    std::scoped_lock lock(other.messagesLock);
    messages = other.messages;
}

void SingleMessageBuffer::push(BufferEntry&& entry, bool keepNewer)
{
    const TopicName topic = entry.msg->getTopic(); //buffer because getTopic does map lookup
    {
        std::scoped_lock lock(messagesLock);

        auto existing = messages.find(topic);
        if(existing != messages.end())
        {
            if(keepNewer && existing->second.receiveTime > entry.receiveTime)
            {
                return;
            }
            existing->second = std::move(entry);
        }
        else
        {
            //no need to check the return, we know that it succeeded because the
            //entry didn't exist before. We just use try_emplace because it
            //has a nicer api
            messages.try_emplace(topic, std::move(entry));
        }
    }
}

void SingleMessageBuffer::writeToBag(rosbag::Bag& bag, const ros::Time& rewriteTimestamp) const
{
    std::scoped_lock lock(messagesLock);

    for(auto& [topic, message] : messages)
    {
        bag.write(topic, rewriteTimestamp, message.msg,
                message.msg->getConnectionHeader());
    }
}

}//end namespace