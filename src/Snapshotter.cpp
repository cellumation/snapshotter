#include "Snapshotter.hpp"


using Subscriber = ros::Subscriber;

namespace snapshotter
{

Snapshotter::Snapshotter(ros::NodeHandle& nh, const Snapshotter::Config& cfg) :
    nh(nh), cfg(cfg), buffer(cfg.maxMemoryBytes)
{

}

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
    /** Both writeToBag() and clear() are thread-safe.
     *  I.e. all other threads will block in topicCB until writing has finished.
     *  In theory a thread could sneak in between the two calls and add a few
     *  messages to the buffer before clear() is called. But this doesn't really
     *  matter.
     *
     * writeToBag() takes a long time to write the bag to disk. During that time
     * the internal buffers of the subscribers will overflow and drop messages.
     * I.e. there will be a gap in the data between two log files.
     */
    buffer.writeToBag(path, compression);
    buffer.clear();
}

void Snapshotter::topicCB(const ros::MessageEvent<ShapeShifterMsg>& msg)
{
    buffer.push(std::move(msg.getConstMessage()), msg.getReceiptTime());
}

}//end namespace