#pragma once
#include <deque>
#include "ShapeShifterMsg.hpp"

namespace snapshotter
{

/** A naive implementation of a ringbuffer to be later replaced by something more efficient */
class RingBuffer
{
public:
    RingBuffer(size_t maxSize) : maxSize(maxSize), currentSize(0) {}

    void push(const ShapeShifterMsg& msg)
    {
        while(currentSize + msg.objectSize() > maxSize)
        {
            currentSize -= buffer.front().objectSize();
            buffer.pop_front();
        }
        currentSize += msg.objectSize();
        //TODO avoid the copy, replace with move
        buffer.push_back(msg);
    }

private:
    size_t maxSize;
    size_t currentSize;
    std::deque<ShapeShifterMsg> buffer;
};


}