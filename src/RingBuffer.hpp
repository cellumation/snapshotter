#pragma once
#include <deque>
#include "ShapeShifterMsg.hpp"
#include <ros/time.h>

namespace snapshotter
{

/** A naive implementation of a ringbuffer to be later replaced by something more efficient */
class RingBuffer
{
public:
    RingBuffer(size_t maxSize) : maxSize(maxSize), currentSize(0) {}

    void push(const ShapeShifterMsg& msg, const ros::Time& receiveTime);

    /**writes the content of the buffer to a bag file */
    void writeToBag(const std::string& path) const;
    /**Removes all entries from this buffer */
    void clear();

private:

    struct BufferEntry
    {
        ShapeShifterMsg msg;
        ros::Time receiveTime;
        BufferEntry(const ShapeShifterMsg& msg, const ros::Time& receiveTime) :
            msg(msg), receiveTime(receiveTime) {}
    };

    size_t maxSize;
    size_t currentSize;
    std::deque<BufferEntry> buffer;
};


}