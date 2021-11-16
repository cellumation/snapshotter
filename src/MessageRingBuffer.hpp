#pragma once
#include <ros/time.h>
#include <deque>
#include <mutex>
#include <functional>
#include "ShapeShifterMsg.hpp"
#include "Common.hpp"

namespace rosbag
{
    class Bag;
}


namespace snapshotter
{

//TODO rename this to something more specific. It is a very specific class :D
/** A simple ring buffer to store ros messages*/
class MessageRingBuffer
{
public:

    /** @param maxSize in bytes
     *  @param droppedCb a callback that is invoked every time a message is
     *                   dropped from this buffer.
     *                   signature: void cb(BufferEntry&& entry) */
    MessageRingBuffer(size_t maxSize, std::function<void(BufferEntry&&)> droppedCb) :
        maxSize(maxSize), droppedCb(droppedCb), currentSize(0) {}

    MessageRingBuffer(const MessageRingBuffer& other);

    /** Add a new message to the ring buffer.
     *  If there is no space left the oldest message will be deleted.
     *  is thread-safe */
    void push(ShapeShifterMsg::ConstPtr msg, const ros::Time& receiveTime);

    /** Writes the content of the buffer to a bag file.
     *  @throw BagWriteException in case of error
     *  is thread-safe */
    void writeToBag(rosbag::Bag& bag) const;

    /** Removes all entries from this buffer.
     *  is thread-safe */
    void clear();

    /** Returns the oldest received-timestamp currently present in the buffer.
     *  is thread-safe */
    ros::Time getOldestReceiveTime() const;

    void setDroppedCb(std::function<void(BufferEntry&&)> cb);

private:

    size_t maxSize; //in bytes //TODO rename to maxSizeBytes
    std::deque<BufferEntry> buffer;
    mutable std::mutex bufferLock;
    std::function<void(BufferEntry&&)> droppedCb;
    size_t currentSize; //in bytes
};

} //end namespace