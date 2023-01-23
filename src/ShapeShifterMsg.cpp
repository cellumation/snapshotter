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
#include "ShapeShifterMsg.hpp"

namespace snapshotter
{
void ShapeShifterMsg::setConnectionHeader(boost::shared_ptr<std::map<std::string, std::string>> header)
{
    connectionHeader = header;
}

const boost::shared_ptr<std::map<std::string, std::string>> ShapeShifterMsg::getConnectionHeader() const
{
    return connectionHeader;
}

// TODO throw if connection header does not contain keys
const std::string& ShapeShifterMsg::getDataType() const
{
    if (connectionHeader)
        return (*connectionHeader)["type"];
    throw std::runtime_error("ShapeShifterMsg connectionHeader not set");
}

const std::string& ShapeShifterMsg::getMD5Sum() const
{
    if (connectionHeader)
        return (*connectionHeader)["md5sum"];
    throw std::runtime_error("ShapeShifterMsg connectionHeader not set");
}

const std::string& ShapeShifterMsg::getMessageDefinition() const
{
    if (connectionHeader)
        return (*connectionHeader)["message_definition"];
    throw std::runtime_error("ShapeShifterMsg connectionHeader not set");
}
const std::string& ShapeShifterMsg::getTopic() const
{
    if (connectionHeader)
        return (*connectionHeader)["topic"];
    throw std::runtime_error("ShapeShifterMsg connectionHeader not set");
}

bool ShapeShifterMsg::getLatching() const
{
    if (connectionHeader)
        return (*connectionHeader)["latching"] == "1";
    throw std::runtime_error("ShapeShifterMsg connectionHeader not set");
}

size_t ShapeShifterMsg::size() const
{
    return data.size();
}

size_t ShapeShifterMsg::objectSize() const
{
    return data.size() + sizeof(ShapeShifterMsg);
}

} // namespace snapshotter