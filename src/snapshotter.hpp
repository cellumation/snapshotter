#pragma once
#include <ros/ros.h>
#include <vector>
#include <topic_tools/shape_shifter.h>
#include <unordered_set>


class Snapshotter
{
public:

    struct Config
    {
        size_t maxMemoryMb;
    };

    Snapshotter(ros::NodeHandle& nh, const Config& cfg);

    /** The snapshotter will subscribe to the given @p topic and log it.
     *  If the topic is already subscribed nothing will happen. */
    void subscribe(const std::string& topic);


private:

    void topicCB(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg);


    ros::NodeHandle& nh;
    Config cfg;

    std::vector<ros::Subscriber> subscribers;

    std::unordered_set<std::string> msgDefs;




};