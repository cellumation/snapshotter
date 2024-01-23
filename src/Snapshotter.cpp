/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Cellumation GmbH
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
#include "Snapshotter.hpp"
#include <cerrno>
#include <chrono>
#include <rmw/rmw.h>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_transport/qos.hpp>
#include <rosbag2_transport/recorder.hpp>
#include <sys/resource.h>
#include <thread>

namespace snapshotter
{
Snapshotter::Snapshotter(rclcpp::Node& nh, const Snapshotter::Config& cfg) :
    nh(nh),
    cfg(cfg),
    buffer(cfg.maxMemoryBytes, [this](BufferEntry&& e) { messageDroppedFromBufferCB(std::move(e)); }),
    log(nh.get_logger())
{}

/// dumps the qos profiles from all endpoints into a yaml node
std::string convertOfferedQosToYaml(const std::vector<rclcpp::TopicEndpointInfo>& endpointInfos)
{
    YAML::Node yaml;
    // FIXME this might create duplicate entries in the yaml file if multiple publishers offer the same qos profile.
    //       This does not cause any problems when replaying the bag, but it is still ugly.
    for (const auto& info : endpointInfos)
    {
        yaml.push_back(rosbag2_transport::Rosbag2QoS(info.qos_profile()));
    }
    return YAML::Dump(yaml);
}

std::string getTopicType(const std::vector<rclcpp::TopicEndpointInfo>& endpointInfos)
{
    std::string type = endpointInfos.front().topic_type();
    for (const auto& info : endpointInfos)
    {
        if (type != info.topic_type())
        {
            throw std::runtime_error("Multiple publishers with different types for the same topic");
        }
    }
    return type;
}

bool Snapshotter::subscribe(const std::string& topic)
{
    if (subscribedTopics.find(topic) == subscribedTopics.end())
    {
        std::vector<rclcpp::TopicEndpointInfo> pubInfo = nh.get_publishers_info_by_topic(topic);
        if (pubInfo.empty())
        {
            RCLCPP_DEBUG_STREAM(log, "No publishers for topic: " << topic
                                                                 << ". Skipping subscription and trying again later");
            return false;
        }

        const rosbag2_storage::TopicMetadata rosMd{.name = topic,
                                                   .type = getTopicType(pubInfo),
                                                   .serialization_format = rmw_get_serialization_format(),
                                                   .offered_qos_profiles = convertOfferedQosToYaml(pubInfo),
                                                   .type_description_hash =
                                                       rosbag2_transport::type_description_hash_for_topic(pubInfo)};

        const bool hasTransientLocal = std::any_of(pubInfo.begin(), pubInfo.end(), [](const auto& info) {
            return info.qos_profile().durability() == rclcpp::DurabilityPolicy::TransientLocal;
        });

        const TopicMetadata md{rosMd, hasTransientLocal, topicMetadata.size()};
        {
            std::lock_guard l(metaDataLock);
            // push before subscription, the callback might access this
            topicMetadata.push_back(md);
        }

        RCLCPP_INFO_STREAM(log, "Subscribing to: " << topic);
        // large buffer because we do not want to miss messages, ever.
        rclcpp::QoS qos(50);
        if (md.transientLocal)
        {
            qos.transient_local();
        }

        rclcpp::SubscriptionBase::SharedPtr sub = nh.create_generic_subscription(
            md.rosMetadata.name, md.rosMetadata.type, qos, [this, md](SerializedMsgPtr msg) { topicCB(msg, md); });

        subscribers.push_back(sub);
        subscribedTopics.emplace(topic);
    }

    return true;
}

void Snapshotter::writeBagFile(const std::string& path, BagCompression compression)
{
    using namespace rosbag2_cpp;

    const auto startTime = std::chrono::steady_clock::now();

    std::unique_ptr<rosbag2_cpp::writer_interfaces::BaseWriterInterface> writerImpl;
    writerImpl = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
    rosbag2_cpp::Writer writer(std::move(writerImpl));

    rosbag2_storage::StorageOptions storageOpts{};
    storageOpts.uri = path;

    switch (compression)
    {
        case BagCompression::FAST:
        {
            storageOpts.storage_preset_profile = "zstd_fast";
        }
        break;
        case BagCompression::SLOW:
        {
            storageOpts.storage_preset_profile = "zstd_small";
        }
        break;
        case BagCompression::NONE:
            break;
        default:
            throw BagWriteException("Unhandled enum value");
    }

    std::unique_ptr<MessageRingBuffer> bufferCopy;
    std::unique_ptr<SingleMessageBuffer> latchedBufferCopy;
    {
        // wait until all threads have stopped writing to the buffers,
        // then copy both buffers
        std::unique_lock lock(writeBagLock);
        bufferCopy = std::make_unique<MessageRingBuffer>(buffer);
        latchedBufferCopy = std::make_unique<SingleMessageBuffer>(lastDroppedLatchedMsgs);
    }

    std::vector<TopicMetadata> metaData;
    {
        std::unique_lock l(metaDataLock);
        metaData = topicMetadata;
    }

    // replace the dropped-callback of the copied buffer. Otherwise dropped messages
    // from that buffer would end up in the original lastDroppedLatchedMsgs buffer.
    bufferCopy->setDroppedCb([](BufferEntry&&) {});

    // writing is done in a seperate thread because errors might happen
    // during writing and there is no guaranteed way to reset the
    // thread priority once an error occurred. If we would use one
    // of the ros threads we would leave a ros thread with very low
    // priority behind in case of error.
    // Even with perfect error handling the call to reset the
    // priority could still fail (and in fact did fail on my machine).
    // Thus it is much easier and safer to spawn a new thread and let
    // it die once we are done writing.
    std::exception_ptr ex = nullptr;
    std::thread t([&] {
        try
        {
            if (cfg.niceOnWrite)
            {
                // according to 'man setpriority' this call is not POSIX conform, and sets the priority
                // for this thread and all of its child threads. This exactly what we want.
                errno = 0;
                if (0 != setpriority(PRIO_PROCESS, 0, 19) || errno != 0)
                {
                    const std::string msg = "setpriority failed: " + std::string(std::strerror(errno));
                    RCLCPP_ERROR_STREAM(log, msg);
                    ex = std::make_exception_ptr(BagWriteException(msg));
                    return;
                }
            }

            writer.open(storageOpts);

            /** write all old latched messages 3 seconds before the actual log starts.
             *  The value 3 is arbitrary. The idea is to make the old latched messages stand out
             *  to a human reader when looking at the bag. We do the calculation in double because rclcpp::Time will
             * throw when the time becomes negative (which can happen when running in simulation because sim time starts
             * at 0) */
            rclcpp::Time latchedTime = bufferCopy->getOldestReceiveTime() - std::chrono::seconds(3);
            latchedTime = std::max(latchedTime, rclcpp::Time(static_cast<int64_t>(0), RCL_ROS_TIME));
            latchedBufferCopy->writeToBag(writer, metaData, latchedTime);

            bufferCopy->writeToBag(writer, metaData);
            writer.close();
        }
        catch (const std::exception& e)
        {
            ex = std::make_exception_ptr(BagWriteException(e.what()));
        }
        catch (...) // we really really don't want to crash :D
        {
            ex = std::make_exception_ptr(BagWriteException("unknown error"));
        }
    });
    t.join();

    const auto elapsedTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - startTime);
    RCLCPP_INFO_STREAM(log, "Writing took: " << elapsedTime.count() << " ms");

    if (ex)
    {
        std::rethrow_exception(ex);
    }
}

void Snapshotter::topicCB(const SerializedMsgPtr& msg, const TopicMetadata& md)
{
    // get time before we aquire the lock, might be more accurate
    rclcpp::Time curTime = nh.now();
    // multiple threads may work on the buffer at the same time
    // as long as no thread is trying to write the buffer to disk
    std::shared_lock lock(writeBagLock);

    buffer.push(msg, curTime, md);
}

void Snapshotter::messageDroppedFromBufferCB(BufferEntry&& entry)
{
    if (!cfg.keepLatched)
    {
        return;
    }
    /** we keep the last dropped message of each latched topic in a seperate buffer.
     *  Upon writing of the bag file the latched message is written a few seconds before
     *  the actual start of the bag file. The timestamp correction is only done for
     *  usability reasons (rqt_bag gui becomes unusable when we dont correct the
     *  timestamp of old latched messages).
     */

    bool isLatched = false;
    {
        std::lock_guard l(metaDataLock);
        isLatched = topicMetadata[entry.topicMetaDataIdx].transientLocal;
    }

    if (isLatched)
    {
        /** In very rare cases newer messages might end up in the lastDroppedLatchedMsgs before older ones.
         *  To avoid overwriting them we keep the newer one inside the buffer.
         *
         *  Newer messages may arrive here before older ones for several reasons:
         *  (1) multiple messages of the same topic are dropped by seperate threads
         *      at nearly the same time from the buffer. Depending on the scheduler those messages
         *      may show up here in the wrong order.
         *  (2) Multiple messages of the same topic arrived in the topicCb at the same time. Depending
         *      on scheduling their order might have been inverted when pushing them to the buffer.
         *  (3) Multiple messages on the same topic originating from different publishers arrive in the
         *      wrong order due to network latency.
         *
         *  Handling all those cases correctly would require a far more complex design of the snapshotter.
         *  I am unwilling to increase design complexity to handle those rare corner cases.
         *  Instead we simply keep the newer of two entries in the lastDroppedLatchedMsgs. This should
         *  catch most of the cases (that where already very rare to begin with). This is good enough.
         **/
        lastDroppedLatchedMsgs.push(std::move(entry), true);
    }
}

} // namespace snapshotter
