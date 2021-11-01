#include "ros/ros.h"
#include "snapshotter.hpp"

using namespace snapshotter;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "snapshotter_test");
    ros::NodeHandle nh("~");

    Snapshotter::Config cfg;
    cfg.maxMemoryBytes = 1 * 1024 * 1024 * 1024;
    Snapshotter snapshotter(nh, cfg);

    boost::function<void (const ros::TimerEvent&)> subscribeTopics =
    [&snapshotter] (const ros::TimerEvent&)
    {
        ros::master::V_TopicInfo allTopics;
        if(ros::master::getTopics(allTopics))
        {
            for(const ros::master::TopicInfo& ti : allTopics)
            {
                snapshotter.subscribe(ti.name);
            }
        }
    };
    ros::Timer topicCheck = nh.createTimer(ros::Duration(2.0),subscribeTopics);

    ros::spin();
}