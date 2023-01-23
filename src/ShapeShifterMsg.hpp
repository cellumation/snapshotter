/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Cellumation GmbH
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************/
#pragma once
#include "ros/console.h"
#include "ros/ros.h"
#include <map>
#include <memory>
#include <ros/message_traits.h>
#include <set>
#include <string>

namespace snapshotter
{
/** A memory efficient re-implementation of the ros topic_tools::ShapeShifter.*/
class ShapeShifterMsg
{
public:
    typedef boost::shared_ptr<ShapeShifterMsg> Ptr;
    typedef boost::shared_ptr<ShapeShifterMsg const> ConstPtr;

    void setConnectionHeader(boost::shared_ptr<std::map<std::string, std::string>> header);
    const boost::shared_ptr<std::map<std::string, std::string>> getConnectionHeader() const;

    /** the following getters use a map lookup internally.*/
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
    data.insert(data.end(), stream.getData(), stream.getData() + stream.getLength());
}

} // namespace snapshotter

// Message traits allow shape shifter to work with the new serialization API
namespace ros::message_traits
{
template<>
struct IsMessage<snapshotter::ShapeShifterMsg> : TrueType
{};
template<>
struct IsMessage<const snapshotter::ShapeShifterMsg> : TrueType
{};

template<>
struct MD5Sum<snapshotter::ShapeShifterMsg>
{
    static const char* value(const snapshotter::ShapeShifterMsg& m) { return m.getMD5Sum().c_str(); }

    // Used statically, a ShapeShifterMsg appears to be of any type
    static const char* value() { return "*"; }
};

template<>
struct DataType<snapshotter::ShapeShifterMsg>
{
    static const char* value(const snapshotter::ShapeShifterMsg& m) { return m.getDataType().c_str(); }

    // Used statically, a ShapeShifterMsg appears to be of any type
    static const char* value() { return "*"; }
};

template<>
struct Definition<snapshotter::ShapeShifterMsg>
{
    static const char* value(const snapshotter::ShapeShifterMsg& m) { return m.getMessageDefinition().c_str(); }
};

} // namespace ros::message_traits

namespace ros::serialization
{
template<>
struct Serializer<snapshotter::ShapeShifterMsg>
{
    template<typename Stream>
    inline static void write(Stream& stream, const snapshotter::ShapeShifterMsg& m)
    {
        m.write(stream);
    }

    template<typename Stream>
    inline static void read(Stream& stream, snapshotter::ShapeShifterMsg& m)
    {
        m.read(stream);
    }

    inline static uint32_t serializedLength(const snapshotter::ShapeShifterMsg& m) { return m.size(); }
};

template<>
struct PreDeserialize<snapshotter::ShapeShifterMsg>
{
    static void notify(const PreDeserializeParams<snapshotter::ShapeShifterMsg>& params)
    {
        params.message->setConnectionHeader(params.connection_header);
    }
};

} // namespace ros::serialization
