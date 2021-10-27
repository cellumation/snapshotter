#include "ros/ros.h"
#include "snapshotter.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "snapshotter_test");
    ros::NodeHandle nh("~");

    Snapshotter::Config cfg;
    cfg.maxMemoryMb = 1000;
    Snapshotter snapshotter(nh, cfg);

    ros::master::V_TopicInfo allTopics;
    if(ros::master::getTopics(allTopics))
    {
        for(const ros::master::TopicInfo& ti : allTopics)
        {
            snapshotter.subscribe(ti.name);
        }
    }

    ros::spin();

}