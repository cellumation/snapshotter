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
#include <mutex>
#include <string>
#include <unordered_map>

namespace rosbag
{
class Bag;
}

namespace snapshotter
{
/** Stores exactly one message per topic.*/
class SingleMessageBuffer
{
public:
    SingleMessageBuffer() = default;
    SingleMessageBuffer(const SingleMessageBuffer& other);

    /** Stores the given entry inside the buffer. Drops existing entry if necessary.
     * @param keepNewer If true the existing entry will only be replaced if it is older than
     *                  @p entry. Otherwise it will always be replaced.
     *  is thread-safe */
    void push(BufferEntry&& entry, bool keepNewer);

    /** Writes the content of the buffer to a bag file.
     *  @param rewriteTimestamp All messages will use this timestamp as receivedTime inside the bag
     *  @throw BagWriteException in case of error
     *  is thread-safe */
    void writeToBag(rosbag::Bag& bag, ros::Time rewriteTimestamp) const;

private:
    using TopicName = std::string;
    std::unordered_map<TopicName, BufferEntry> messages;
    mutable std::mutex messagesLock;
};

} // namespace snapshotter