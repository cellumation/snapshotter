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
        /**ATTENTION The BufferEntries are copied when writting to the file.
         *           This becomes a problem when the BufferEntry grows in size.
         *           Currently this is fine because it only contains a pointer and two ints.
         *           DO NOT ADD MORE ATTRIBUTES! (or if you do, rethink how data is written
         *           to the bag file) */


        ShapeShifterMsg::ConstPtr msg;
        ros::Time receiveTime;
        BufferEntry(ShapeShifterMsg::ConstPtr msg, const ros::Time& receiveTime) :
            msg(std::move(msg)), receiveTime(receiveTime) {}
    };

}