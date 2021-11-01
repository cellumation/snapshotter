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

//TODO throw if connection header does not contain keys
const std::string& ShapeShifterMsg::getDataType() const
{
    if(connectionHeader)
        return (*connectionHeader)["type"];
    throw std::runtime_error("ShapeShifterMsg connectionHeader not set");
}

const std::string& ShapeShifterMsg::getMD5Sum() const
{
    if(connectionHeader)
        return (*connectionHeader)["md5sum"];
    throw std::runtime_error("ShapeShifterMsg connectionHeader not set");
}

const std::string& ShapeShifterMsg::getMessageDefinition() const
{
    if(connectionHeader)
        return (*connectionHeader)["message_definition"];
    throw std::runtime_error("ShapeShifterMsg connectionHeader not set");
}
const std::string& ShapeShifterMsg::getTopic() const
{
    if(connectionHeader)
        return (*connectionHeader)["topic"];
    throw std::runtime_error("ShapeShifterMsg connectionHeader not set");
}

bool ShapeShifterMsg::getLatching() const
{
    if(connectionHeader)
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

} //end namespace