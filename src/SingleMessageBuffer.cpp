#include "SingleMessageBuffer.hpp"
#include <rosbag/bag.h>

namespace snapshotter
{

void SingleMessageBuffer::push(BufferEntry&& entry)
{
    const TopicName topic = entry.msg->getTopic(); //buffer because getTopic does map lookup
    {
        std::scoped_lock lock(messagesLock);

        auto [iter, inserted] = messages.try_emplace(topic, std::move(entry));
        if(!inserted)
        {
            //replace existing entry if already exist
            iter->second = std::move(entry);
        }
    }
}

void SingleMessageBuffer::writeToBag(rosbag::Bag& bag, const ros::Time& rewriteTimestamp) const
{
    std::scoped_lock lock(messagesLock);
    for(auto& [topic, message] : messages)
    {
        bag.write(topic, rewriteTimestamp, message.msg,
                message.msg->getConnectionHeader());
    }
}

}//end namespace