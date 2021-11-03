#include "RingBuffer.hpp"
#include <rosbag/bag.h>

namespace snapshotter
{
void RingBuffer::push(ShapeShifterMsg::ConstPtr msg, const ros::Time& receiveTime)
{
    const size_t entrySize = msg->objectSize() + sizeof(ros::Time);

    std::scoped_lock lock(bufferLock);

    while(currentSize + entrySize > maxSize)
    {
        currentSize -= buffer.front().msg->objectSize() + sizeof(ros::Time);
        buffer.pop_front();
    }
    currentSize += entrySize;
    buffer.emplace_back(std::move(msg), receiveTime);
}

void RingBuffer::writeToBag(const std::string& filepath,
                            BagCompression compression) const
{
    //the buffer should not be modified while writing
    std::scoped_lock lock(bufferLock);

    using namespace rosbag;
    Bag bag;

    switch(compression)
    {
    case BagCompression::FAST:
        bag.setCompression(CompressionType::LZ4);
        break;
    case BagCompression::SLOW:
        bag.setCompression(CompressionType::BZ2);
        break;
    case BagCompression::NONE:
        break;
    default:
        throw BagWriteException("Unhandled enum value");
    }

    try
    {
        bag.open(filepath, bagmode::Write);
        for(const BufferEntry& entry : buffer)
        {
            bag.write(entry.msg->getTopic(), entry.receiveTime, entry.msg,
                    entry.msg->getConnectionHeader());
        }
        bag.close();
    }
    catch(const std::exception& ex)
    {
        throw BagWriteException(ex.what());
    }
    catch(...) //we really really don't want to crash :D
    {
        throw BagWriteException("unknown error");
    }
}

void RingBuffer::clear()
{
    std::scoped_lock lock(bufferLock); //nobody should write to the buffer while clearing
    buffer.clear();
    currentSize = 0;
}

}