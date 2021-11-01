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

/** A memory efficient re-implementation of the ros topic_tools::ShapeShifter.
 * The original shape_shifter creates a copy of the connection_header. This leads to
 * lots of memory overhead because the connection_header rarely changes while a connection
 * is open.
 * Instead of copying the connection_header this implementation retains a shared_ptr to the
 * original connection_header provided during deserialization. The connection_header is
 * re-used for all messages of the same connection.
 */
class ShapeShifterMsg
{
public:

    typedef boost::shared_ptr<ShapeShifterMsg> Ptr;
    typedef boost::shared_ptr<ShapeShifterMsg const> ConstPtr;


    void setConnectionHeader(boost::shared_ptr<std::map<std::string, std::string>> header);
    const boost::shared_ptr<std::map<std::string, std::string>> getConnectionHeader() const;


    const std::string& getDataType() const;
    const std::string& getMD5Sum() const;
    const std::string& getMessageDefinition() const;
    const std::string& getTopic() const;
    bool getLatching() const;



    /**Write serialized message contents out to a stream */
    template<typename Stream>
    void write(Stream& stream) const;

    /**Read data from stream into this message */
    template<typename Stream>
    void read(Stream& stream);

    //! Return the size of the serialized message
    size_t size() const;

    /** Return the size that this object consumes in memory */
    size_t objectSize() const;

    void setReceiptTime(const ros::Time& time);
    const ros::Time& getReceiptTime() const;

private:

    boost::shared_ptr<std::map<std::string, std::string>> connectionHeader;
    std::vector<uint8_t> data;
};

template<typename Stream>
void ShapeShifterMsg::write(Stream& stream) const
{
  if (data.size() > 0)
    memcpy(stream.advance(data.size()), data.data(), data.size());
}

template<typename Stream>
void ShapeShifterMsg::read(Stream& stream)
{
  data.resize(stream.getLength());
  memcpy(data.data(), stream.getData(), stream.getLength());
}

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
        return m.getMD5Sum().c_str();
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
        return m.getDataType().c_str();
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
        return m.getMessageDefinition().c_str();
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
        m.write(stream);
    }

    template <typename Stream>
    inline static void read(Stream& stream, snapshotter::ShapeShifterMsg& m)
    {
        m.read(stream);
    }

    inline static uint32_t serializedLength(const snapshotter::ShapeShifterMsg& m)
    {
        return m.size();
    }
};

template <>
struct PreDeserialize<snapshotter::ShapeShifterMsg>
{
    static void notify(const PreDeserializeParams<snapshotter::ShapeShifterMsg>& params)
    {
        params.message->setConnectionHeader(params.connection_header);
    }
};

}
