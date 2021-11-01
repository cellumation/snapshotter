#include "RingBuffer.hpp"
#include <rosbag/bag.h>

namespace snapshotter
{
void RingBuffer::push(const ShapeShifterMsg& msg, const ros::Time& receiveTime)
{
    const size_t entrySize = msg.objectSize() + sizeof(ros::Time);
    while(currentSize + entrySize > maxSize)
    {
        currentSize -= buffer.front().msg.objectSize() + sizeof(ros::Time);
        buffer.pop_front();
    }
    currentSize += entrySize;
    //TODO avoid the copy, replace with move
    buffer.emplace_back(msg, receiveTime);
}

void RingBuffer::writeToBag(const std::string& filepath) const
{
    using namespace rosbag;
    Bag bag;
    //TODO  compression parameter
    // bag.setCompression(CompressionType::BZ2);
    bag.open(filepath, bagmode::Write); //TODO catch exceptions etc.

    for(const BufferEntry& entry : buffer)
    {
        bag.write(entry.msg.getTopic(), entry.receiveTime, entry.msg,
                  entry.msg.getConnectionHeader());
    }
    bag.close();
}

void RingBuffer::clear()
{
    buffer.clear();
    currentSize = 0;
}

}