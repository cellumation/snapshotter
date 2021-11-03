#pragma once
#include <ros/ros.h>
#include <vector>
#include <topic_tools/shape_shifter.h>
#include <unordered_set>
#include "ShapeShifterMsg.hpp"
#include "RingBuffer.hpp"

namespace snapshotter
{
class Snapshotter
{
public:

    struct Config
    {
        size_t maxMemoryBytes;
    };

    Snapshotter(ros::NodeHandle& nh, const Config& cfg);

    /** The snapshotter will subscribe to the given @p topic and log it.
     *  If the topic is already subscribed nothing will happen. */
    void subscribe(const std::string& topic);

    /**Stops recording, writes the bag and restarts recording
     * @throw std::runtime_error on error     */
    void writeBagFile(const std::string& path);


private:

    void topicCB(const ros::MessageEvent<ShapeShifterMsg>& msg);


    ros::NodeHandle& nh;
    Config cfg;
    std::vector<ros::Subscriber> subscribers;
    std::unordered_set<std::string> subscribedTopics;
    RingBuffer buffer;
    bool stopRecording = false;/**if true no topics will be recorded */

};

}//end namespace