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
#include <rosbag/bag.h>

using Subscriber = ros::Subscriber;

namespace snapshotter
{

Snapshotter::Snapshotter(ros::NodeHandle& nh, const Snapshotter::Config& cfg) :
    nh(nh), cfg(cfg),
    buffer(cfg.maxMemoryBytes,
           [this](BufferEntry&& e){messageDroppedFromBufferCB(std::move(e));})
{}

void Snapshotter::subscribe(const std::string& topic)
{
    if(subscribedTopics.find(topic) == subscribedTopics.end())
    {
        ROS_INFO_STREAM("Subscribing to: " << topic);

        boost::shared_ptr<Subscriber> sub(boost::make_shared<Subscriber>());
        ros::SubscribeOptions ops;
        ops.topic = topic;
        ops.queue_size = 50;
        ops.md5sum = ros::message_traits::md5sum<ShapeShifterMsg>();
        ops.datatype = ros::message_traits::datatype<ShapeShifterMsg>();
        ops.helper = boost::make_shared<ros::SubscriptionCallbackHelperT<const ros::MessageEvent<ShapeShifterMsg>&> >(
                boost::bind(&Snapshotter::topicCB, this, _1));

        subscribers.push_back(nh.subscribe(ops));
        subscribedTopics.emplace(topic);
    }
}

void Snapshotter::writeBagFile(const std::string& path, BagCompression compression)
{
    using namespace rosbag;
    Bag bag;

    switch(compression)
    {
    case BagCompression::FAST:
        bag.setCompression(CompressionType::LZ4);
        break;
    case BagCompression::SLOW:
        bag.setCompression(CompressionType::BZ2);
        break;
    case BagCompression::NONE:
        break;
    default:
        throw BagWriteException("Unhandled enum value");
    }

    try
    {
        std::unique_ptr<MessageRingBuffer> bufferCopy;
        std::unique_ptr<SingleMessageBuffer> latchedBufferCopy;
        {
            //wait until all threads have stopped writing to the buffers,
            //then copy both buffers
            std::unique_lock lock(writeBagLock);
            bufferCopy.reset(new MessageRingBuffer(buffer));
            latchedBufferCopy.reset(new SingleMessageBuffer(lastDroppedLatchedMsgs));
        }

        //replace the dropped-callback of the copied buffer. Otherwise dropped messages
        //from that buffer would end up in the original lastDroppedLatchedMsgs buffer.
        bufferCopy->setDroppedCb([](BufferEntry&&){});

        bag.open(path, bagmode::Write);
        /** write all old latched messages 3 seconds before the actual log starts.
         *  The value 3 is arbitrary. The idea is to make the old latched messages stand out
         *  to a human reader when looking at the bag. */
        const ros::Time latchedTime = bufferCopy->getOldestReceiveTime() - ros::Duration(3);
        latchedBufferCopy->writeToBag(bag, latchedTime);

        bufferCopy->writeToBag(bag);
        bag.close();
    }
    catch(const std::exception& ex)
    {
        throw BagWriteException(ex.what());
    }
    catch(...) //we really really don't want to crash :D
    {
        throw BagWriteException("unknown error");
    }
}

void Snapshotter::topicCB(const ros::MessageEvent<ShapeShifterMsg>& msg)
{
    //multiple threads may work on the buffer at the same time
    //as long as no thread is trying to write the buffer to disk
    std::shared_lock lock(writeBagLock);

    buffer.push(std::move(msg.getConstMessage()), msg.getReceiptTime());
}

void Snapshotter::messageDroppedFromBufferCB(BufferEntry&& entry)
{
    if(!cfg.keepLatched)
    {
        return;
    }
    /** we keep the last dropped message of each latched topic in a seperate buffer.
     *  Upon writing of the bag file the latched message is written a few seconds before
     *  the actual start of the bag file. The timestamp correction is only done for
     *  usability reasons (rqt_bag gui becomes unusable when we dont correct the
     *  timestamp of old latched messages).
     */

    //NOTE getLatching call does a map lookup. if we need to call this more than once we should buffer
    if(entry.msg->getLatching())
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


}//end namespace