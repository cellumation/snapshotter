#pragma once
#include <ros/ros.h>
#include <vector>
#include <topic_tools/shape_shifter.h>
#include <unordered_set>
#include "ShapeShifterMsg.hpp"
#include "MessageRingBuffer.hpp"
#include "SingleMessageBuffer.hpp"
#include "Common.hpp"
#include <shared_mutex>

namespace snapshotter
{
class Snapshotter
{
public:

    struct Config
    {
        size_t maxMemoryBytes;
        /** If true the last message of each latched topic will be kept inside the buffer.
         */
        bool keepLatched;
    };

    Snapshotter(ros::NodeHandle& nh, const Config& cfg);

    /** The snapshotter will subscribe to the given @p topic and log it.
     *  If the topic is already subscribed nothing will happen. */
    void subscribe(const std::string& topic);

    /**Stops recording, writes the bag and restarts recording
     * @throw BagWriteException in case of error */
    void writeBagFile(const std::string& path, BagCompression compression);


private:

    void topicCB(const ros::MessageEvent<ShapeShifterMsg>& msg);

    /** is invoked every time a message is dropped from the MessageRingBuffer */
    void messageDroppedFromBufferCB(BufferEntry&& entry);

    ros::NodeHandle& nh;
    Config cfg;
    std::vector<ros::Subscriber> subscribers;
    std::unordered_set<std::string> subscribedTopics;
    MessageRingBuffer buffer;
    SingleMessageBuffer lastDroppedLatchedMsgs;
    std::shared_mutex writeBagLock;


};

}//end namespace