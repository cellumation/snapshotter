#pragma once
#include <deque>
#include "ShapeShifterMsg.hpp"
#include <ros/time.h>
#include <mutex>

namespace snapshotter
{

/** A naive implementation of a ringbuffer to be later replaced by something more efficient */
class RingBuffer
{
public:

    RingBuffer(size_t maxSize) : maxSize(maxSize), currentSize(0) {}

    void push(ShapeShifterMsg::ConstPtr msg, const ros::Time& receiveTime);

    /**writes the content of the buffer to a bag file */
    void writeToBag(const std::string& path) const;
    /**Removes all entries from this buffer */
    void clear();

private:

    struct BufferEntry
    {
        ShapeShifterMsg::ConstPtr msg;
        ros::Time receiveTime;
        BufferEntry(ShapeShifterMsg::ConstPtr msg, const ros::Time& receiveTime) :
            msg(std::move(msg)), receiveTime(receiveTime) {}
    };


    size_t maxSize;
    size_t currentSize;
    std::deque<BufferEntry> buffer;
    std::mutex bufferLock;
};


}