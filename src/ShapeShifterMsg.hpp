#pragma once
#include "ros/ros.h"
#include "ros/console.h"
#include <ros/message_traits.h>
#include <set>
#include <map>
#include <memory>
#include <string>

namespace snapshotter
{

/** A more memory efficient re-implementation of the ros topic_tools::ShapeShifter.
 * The original shape_shifter creates a copy of the connection_header. This leads to
 * lots of memory overhead because the connection_header rarely changes while a connection
 * is open.
 * Instead of copying the connection_header this implementation retains a shared_ptr to the
 * original connection_header provided during deserialization.
 */
class ShapeShifterMsg
{
    typedef boost::shared_ptr<ShapeShifterMsg> Ptr;
    typedef boost::shared_ptr<ShapeShifterMsg const> ConstPtr;
};
}

// Message traits allow shape shifter to work with the new serialization API
namespace ros::message_traits
{

template<>
struct IsMessage<snapshotter::ShapeShifterMsg> : TrueType
{
};
template<>
struct IsMessage<const snapshotter::ShapeShifterMsg> : TrueType
{
};

template<>
struct MD5Sum<snapshotter::ShapeShifterMsg>
{
    static const char* value(const snapshotter::ShapeShifterMsg& m)
    {
        return "";
        //TODO
        // return m.getMD5Sum().c_str();
    }

    // Used statically, a ShapeShifterMsg appears to be of any type
    static const char* value()
    {
        return "*";
    }
};

template<>
struct DataType<snapshotter::ShapeShifterMsg>
{
    static const char* value(const snapshotter::ShapeShifterMsg& m)
    {
        return "";
        //TODO
        // return m.getDataType().c_str();
    }

    // Used statically, a ShapeShifterMsg appears to be of any type
    static const char* value()
    {
        return "*";
    }
};

template <>
struct Definition<snapshotter::ShapeShifterMsg>
{
    static const char* value(const snapshotter::ShapeShifterMsg& m)
    {
        return "";
        //TODO
        //return m.getMessageDefinition().c_str();
    }
};

}

namespace ros::serialization
{

template <>
struct Serializer<snapshotter::ShapeShifterMsg>
{
    template <typename Stream>
    inline static void write(Stream& stream, const snapshotter::ShapeShifterMsg& m)
    {
        //TODO
        // m.write(stream);
    }

    template <typename Stream>
    inline static void read(Stream& stream, snapshotter::ShapeShifterMsg& m)
    {
        //TODO
        // m.read(stream);
    }

    inline static uint32_t serializedLength(const snapshotter::ShapeShifterMsg& m)
    {
        //TODO
        return 0;
        // return m.size();
    }
};

template <>
struct PreDeserialize<snapshotter::ShapeShifterMsg>
{
    static void notify(const PreDeserializeParams<snapshotter::ShapeShifterMsg>& params)
    {
        static std::set<boost::shared_ptr<std::map<std::string, std::string>>> headers;

        headers.emplace(params.connection_header);
        ROS_INFO_STREAM("notify: " << headers.size());

        // std::string md5 = (*params.connection_header)["md5sum"];
        // std::string datatype = (*params.connection_header)["type"];
        // std::string msg_def = (*params.connection_header)["message_definition"];
        // std::string latching = (*params.connection_header)["latching"];

        // params.message->morph(md5, datatype, msg_def, latching);
    }
};

}
