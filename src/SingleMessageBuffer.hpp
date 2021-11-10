#pragma once
#include "Common.hpp"
#include <unordered_map>
#include <string>
#include <mutex>

namespace rosbag
{
    class Bag;
}

namespace snapshotter
{
/** Stores exactly one message per topic.*/
class SingleMessageBuffer
{
public:
    /** Stores the given entry inside the buffer. Drops existing entry if necessary.
     * @param keepNewer If true the existing entry will only be replaced if it is older than
     *                  @p entry. Otherwise it will always be replaced.
     *  is thread-safe */
    void push(BufferEntry&& entry, bool keepNewer);

    /** Writes the content of the buffer to a bag file.
     *  @param rewriteTimestamp All messages will use this timestamp as receivedTime inside the bag
     *  @throw BagWriteException in case of error
     *  is thread-safe */
    void writeToBag(rosbag::Bag& bag, const ros::Time& rewriteTimestamp) const;

private:
    using TopicName = std::string;
    std::unordered_map<TopicName, BufferEntry> messages;
    mutable std::mutex messagesLock;
};

}//end namespace