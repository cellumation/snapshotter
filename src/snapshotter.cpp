#include "snapshotter.hpp"


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
        ops.helper = boost::make_shared<ros::SubscriptionCallbackHelperT<const ros::MessageEvent<ShapeShifterMsg const>&> >(
                boost::bind(&Snapshotter::topicCB, this, _1));

        subscribers.push_back(nh.subscribe(ops));
        subscribedTopics.emplace(topic);
    }
}

void Snapshotter::writeBagFile(const std::string& path)
{
    //TODO check path etc.
    stopRecording = true;
    //TODO if multi threaded we need to wait for all write threads to end here
    buffer.writeToBag(path);
    buffer.clear();

    stopRecording = false;
}

void Snapshotter::topicCB(const ros::MessageEvent<ShapeShifterMsg const>& msg)
{
    if(!stopRecording)
    {
        buffer.push(*msg.getConstMessage(), msg.getReceiptTime());
    }
}

}//end namespace