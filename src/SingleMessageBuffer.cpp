#include "SingleMessageBuffer.hpp"
#include <rosbag/bag.h>

namespace snapshotter
{

void SingleMessageBuffer::push(BufferEntry&& entry, bool keepNewer)
{
    const TopicName topic = entry.msg->getTopic(); //buffer because getTopic does map lookup
    {
        std::scoped_lock lock(messagesLock);

        auto existing = messages.find(topic);
        if(existing != messages.end())
        {
            if(keepNewer && existing->second.receiveTime > entry.receiveTime)
            {
                return;
            }
            existing->second = std::move(entry);
        }
        else
        {
            //no need to check the return, we know that it succeeded because the
            //entry didn't exist before. We just use try_emplace because it
            //has a nicer api
            messages.try_emplace(topic, std::move(entry));
        }
    }
}

void SingleMessageBuffer::writeToBag(rosbag::Bag& bag, const ros::Time& rewriteTimestamp) const
{
    std::unordered_map<TopicName, BufferEntry> messageCopy;
    {
        std::scoped_lock lock(messagesLock);
        messageCopy = messages;
    }

    for(auto& [topic, message] : messageCopy)
    {
        bag.write(topic, rewriteTimestamp, message.msg,
                message.msg->getConnectionHeader());
    }
}

}//end namespace