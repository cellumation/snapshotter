#pragma once
#include <stdexcept>
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

}