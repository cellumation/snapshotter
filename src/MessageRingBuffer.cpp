#include "MessageRingBuffer.hpp"
#include <rosbag/bag.h>

namespace snapshotter
{
void MessageRingBuffer::push(ShapeShifterMsg::ConstPtr msg, const ros::Time& receiveTime)
{
    const size_t entrySize = msg->objectSize() + sizeof(ros::Time);
    std::vector<BufferEntry> droppedMsgs;
    {
        std::scoped_lock lock(bufferLock);

        while(currentSize + entrySize > maxSize)
        {
            //we accumulate all dropped messages to invoke the droppedCallback after unlocking.
            //The callback might take a long time and potentially lock other resources causing a deadlock
            currentSize -= buffer.front().msg->objectSize() + sizeof(ros::Time);
            droppedMsgs.emplace_back(std::move(buffer.front()));
            buffer.pop_front();
        }
        currentSize += entrySize;
        buffer.emplace_back(std::move(msg), receiveTime);
    }

    for(BufferEntry& e : droppedMsgs)
    {
        droppedCb(std::move(e));
    }
}

void MessageRingBuffer::writeToBag(rosbag::Bag& bag) const
{
    std::scoped_lock lock(bufferLock);
    for(const BufferEntry& entry : buffer)
    {
        bag.write(entry.msg->getTopic(), entry.receiveTime, entry.msg,
                entry.msg->getConnectionHeader());
    }
}

void MessageRingBuffer::clear()
{
    std::scoped_lock lock(bufferLock);
    for(const BufferEntry& entry : buffer)
    {
        droppedCb(std::move(buffer.front()));
        buffer.pop_front();
    }
    currentSize = 0;
}

ros::Time MessageRingBuffer::getOldestReceiveTime() const
{
    std::scoped_lock lock(bufferLock);
    ros::Time oldestTime = ros::Time::now() + ros::Duration(10); //start with a timestamp in the future
    for(const BufferEntry& entry : buffer)
    {
        oldestTime = std::min(oldestTime, entry.receiveTime);
    }
    return oldestTime;
}

}