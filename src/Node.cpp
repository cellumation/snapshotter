#include "Node.hpp"

#include <chrono>
#include <rcl/service_introspection.h>
#include <rosbag2_cpp/service_utils.hpp>

namespace snapshotter
{

SnapshotNode::SnapshotNode(rclcpp::Node& nh, const snapshotter::Snapshotter::Config& cfg, BagCompression compression,
                           const TopicFilter& topicFilter) :
    nh{nh},
    compression{compression},
    topicFilter{topicFilter},
    snapshotter{nh, cfg}
{
    subscribeTopics();

    subscribeTimer = nh.create_wall_timer(std::chrono::seconds(2), [this]() { subscribeTopics(); });

    service = nh.create_service<snapshotter::srv::TakeSnapshot>(
        "take_snapshot", [&](const std::shared_ptr<rmw_request_id_t> header,
                             snapshotter::srv::TakeSnapshot::Request::SharedPtr req) { handleRequest(header, req); });
    service->configure_introspection(nh.get_clock(), rclcpp::SystemDefaultsQoS(), RCL_SERVICE_INTROSPECTION_CONTENTS);
}

void SnapshotNode::handleRequest(const std::shared_ptr<rmw_request_id_t> header,
                                 const std::shared_ptr<snapshotter::srv::TakeSnapshot::Request> req)
{
    if (!takeSnapshotServiceLock.try_lock())
    {
        snapshotter::srv::TakeSnapshot::Response resp;
        resp.message = "Already taking snapshot";
        resp.success = false;
        service->send_response(*header, resp);
        return;
    }

    snapshotter.writeBagFile(req->filename, compression, [this, header](const std::optional<BagWriteException>& error) {
        snapshotter::srv::TakeSnapshot::Response resp;
        resp.success = !error.has_value();
        if (error)
        {
            resp.message = error->what();
        }
        try
        {
            service->send_response(*header, resp);
        }
        catch (...)
        {
            // catch everything because we need to make sure that the mutex is always unlocked
            RCLCPP_ERROR_STREAM(nh.get_logger(), "Failed to send service response");
        }
        takeSnapshotServiceLock.unlock();
    });
}

void SnapshotNode::subscribeTopics()
{
    std::map<std::string, std::vector<std::string>> allTopics = nh.get_topic_names_and_types();

    for (const auto& [topicName, topicTypes] : allTopics)
    {
        if (topicTypes.empty())
        {
            RCLCPP_ERROR_STREAM(nh.get_logger(), "Topic " << topicName << " has no type information, skipping");
            continue;
        }
        if (rosbag2_cpp::is_service_event_topic(topicName, topicTypes[0]))
        {
            const std::string serviceName = rosbag2_cpp::service_event_topic_name_to_service_name(topicName);
            if (!topicFilter.excludeService(serviceName))
            {
                snapshotter.subscribe(topicName);
            }
        }
        else if (!topicFilter.exclude(topicName))
        {
            snapshotter.subscribe(topicName);
        }
    }
}

} // namespace snapshotter
