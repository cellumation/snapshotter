#pragma once
#include <stdexcept>
#include "ShapeShifterMsg.hpp"

namespace snapshotter
{
    enum class BagCompression
    {
        NONE,
        SLOW, //good compression but takes a long time
        FAST  //a little bit of compression but fast
    };

    struct BagWriteException : std::runtime_error
    {
        using std::runtime_error::runtime_error;
    };

    struct BufferEntry
    {
        ShapeShifterMsg::ConstPtr msg;
        ros::Time receiveTime;
        BufferEntry(ShapeShifterMsg::ConstPtr msg, const ros::Time& receiveTime) :
            msg(std::move(msg)), receiveTime(receiveTime) {}
    };

}