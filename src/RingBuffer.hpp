#pragma once
#include <deque>
#include <ros/time.h>
#include <mutex>
#include "ShapeShifterMsg.hpp"
#include "Common.hpp"


namespace snapshotter
{

/** A simple ring buffer to store ros messages*/
class RingBuffer
{
public:

    /** @param maxSize in bytes */
    RingBuffer(size_t maxSize) : maxSize(maxSize), currentSize(0) {}

    /** Add a new message to the ring buffer.
     *  If there is no space left the oldest message will be deleted.
     *  is thread-safe */
    void push(ShapeShifterMsg::ConstPtr msg, const ros::Time& receiveTime);

    /** Writes the content of the buffer to a bag file.
     *  @throw BagWriteException in case of error
     *  is thread-safe */
    void writeToBag(const std::string& path, BagCompression compression) const;

    /** Removes all entries from this buffer.
     *  is thread-safe */
    void clear();

private:

    struct BufferEntry
    {
        ShapeShifterMsg::ConstPtr msg;
        ros::Time receiveTime;
        BufferEntry(ShapeShifterMsg::ConstPtr msg, const ros::Time& receiveTime) :
            msg(std::move(msg)), receiveTime(receiveTime) {}
    };

    size_t maxSize; //in bytes
    size_t currentSize; //in bytes
    std::deque<BufferEntry> buffer;
    mutable std::mutex bufferLock;
};

} //end namespace