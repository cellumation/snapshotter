#include "snapshotter.hpp"

#include <topic_tools/shape_shifter.h>

using Subscriber = ros::Subscriber;

namespace snapshotter
{

Snapshotter::Snapshotter(ros::NodeHandle& nh, const Snapshotter::Config& cfg) :
    nh(nh), cfg(cfg), buffer(cfg.maxMemoryBytes)
{

}

void Snapshotter::subscribe(const std::string& topic)
{
  ROS_INFO_STREAM("Subscribing to: " << topic);

  boost::shared_ptr<Subscriber> sub(boost::make_shared<Subscriber>());
  ros::SubscribeOptions ops;
  ops.topic = topic;
  ops.queue_size = 10;
  ops.md5sum = ros::message_traits::md5sum<ShapeShifterMsg>();
  ops.datatype = ros::message_traits::datatype<ShapeShifterMsg>();
//   ops.helper = boost::make_shared<ros::SubscriptionCallbackHelperT<const ros::MessageEvent<topic_tools::ShapeShifter const>&> >(
//           boost::bind(&Snapshotter::topicCB, this, _1, queue));

  ops.helper = boost::make_shared<ros::SubscriptionCallbackHelperT<const ros::MessageEvent<ShapeShifterMsg const>&> >(
           boost::bind(&Snapshotter::topicCB, this, _1));

  subscribers.push_back(nh.subscribe(ops));
}


void Snapshotter::topicCB(const ros::MessageEvent<ShapeShifterMsg const>& msg)
{
    buffer.push(*msg.getConstMessage());
}

}//end namespace