#pragma once
#include "Snapshotter.hpp"
#include "TopicFilter.hpp"
#include <memory>
#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>
#include <snapshotter/srv/take_snapshot.hpp>

namespace snapshotter
{

class SnapshotNode
{
public:
    SnapshotNode(rclcpp::Node& nh, const snapshotter::Snapshotter::Config& cfg, BagCompression compression,
                 const TopicFilter& topicFilter);

    void handleRequest(const std::shared_ptr<rmw_request_id_t> header,
                       const std::shared_ptr<snapshotter::srv::TakeSnapshot::Request> req);

private:
    void subscribeTopics();

    Snapshotter snapshotter;
    rclcpp::Node& nh;
    BagCompression compression;
    TopicFilter topicFilter;

    rclcpp::Service<snapshotter::srv::TakeSnapshot>::SharedPtr service;
    std::mutex takeSnapshotServiceLock;
    std::shared_ptr<rclcpp::TimerBase> subscribeTimer;
};

} // namespace snapshotter
