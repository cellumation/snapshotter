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
        /** Both writeToBag() and clear() are thread-safe.
         *  In theory a thread could sneak in between the two calls and add a few
         *  messages to the buffer before clear() is called. But this doesn't really
         *  matter.
         *
         * writeToBag() takes a long time to write the bag to disk. During that time
         * the internal buffers of the subscribers will overflow and drop messages.
         * I.e. there will be a gap in the data between two log files.
         */

        bag.open(path, bagmode::Write);

        //write all old latched messages 3 seconds before the actual log starts.
        //The 3 is arbitrary. The idea is to make the old latched messages stand out
        //to a human reader when looking at the bag.
        const ros::Time latchedTime = buffer.getOldestReceiveTime() - ros::Duration(3);
        lastDroppedLatchedMsgs.writeToBag(bag, latchedTime);

        buffer.writeToBag(bag);
        buffer.clear();//this triggers the messageDroppedFromBufferCB() which in turn will modify lastDroppedLatchedMsgs

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
        lastDroppedLatchedMsgs.push(std::move(entry));
    }
}


}//end namespace