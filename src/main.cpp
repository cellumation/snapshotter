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
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THu
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
#include "TopicFilter.hpp"

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <snapshotter/srv/take_snapshot.hpp>

#include <mutex>
#include <string>
#include <vector>

using namespace snapshotter;

BagCompression getCompression(rclcpp::Node& nh)
{
    std::string compression = nh.get_parameter("bag_compression").as_string();

    if (compression == "none")
    {
        return BagCompression::NONE;
    }
    if (compression == "slow")
    {
        return BagCompression::SLOW;
    }
    if (compression == "fast")
    {
        return BagCompression::FAST;
    }
    throw std::runtime_error("Parameter bag_compression has illegal value: '" + compression + "'");
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions ops;
    ops.automatically_declare_parameters_from_overrides(true);

    rclcpp::Node nh("snapshotter", ops);

    std::vector<std::string> excludeRegexes = nh.get_parameter("exclude_topics").as_string_array();
    std::vector<std::string> includeRegexes; // = nh.get_parameter("include_topics").as_string_array();
    TopicFilter topicFilter(excludeRegexes, includeRegexes);

    Snapshotter::Config cfg;

    int maxMemoryMb = nh.get_parameter("max_memory_mb").as_int();
    if (maxMemoryMb < 0)
    {
        throw std::runtime_error("Parameter 'maxMemoryMb' must be > 0");
    }
    cfg.maxMemoryBytes = size_t(maxMemoryMb) * size_t(1024 * 1024);
    cfg.niceOnWrite = nh.get_parameter("nice_on_write").as_bool();

    Snapshotter snapshotter(nh, cfg);

    std::function<void()> subscribeTopics = [&snapshotter, &topicFilter, &nh]() {
        std::map<std::string, std::vector<std::string>> allTopics = nh.get_topic_names_and_types();

        for (const auto& [topicName, _] : allTopics)
        {
            if (!topicFilter.exclude(topicName))
            {
                snapshotter.subscribe(topicName);
            }
        }
    };
    // call once to immediately subscribe to all available topics
    subscribeTopics();
    auto topicCheck = nh.create_wall_timer(std::chrono::seconds(2), subscribeTopics);

    BagCompression compression = getCompression(nh);

    std::mutex takeSnapshotServiceLock;

    std::function<bool(snapshotter::srv::TakeSnapshot::Request::SharedPtr,
                       snapshotter::srv::TakeSnapshot::Response::SharedPtr)>
        takeSnapshotCb = [&](snapshotter::srv::TakeSnapshot::Request::SharedPtr req,
                             snapshotter::srv::TakeSnapshot::Response::SharedPtr resp) {
            std::unique_lock<std::mutex> lock(takeSnapshotServiceLock, std::try_to_lock);
            if (!lock.owns_lock())
            {
                // mutex wasn't locked. Handle it.
                resp->success = false;
                resp->message = "Already taking a snapshot.";
                return true;
            }

            try
            {
                snapshotter.writeBagFile(req->filename, compression);
                resp->success = true;
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR_STREAM(nh.get_logger(), "Got exception while writing bag : " << e.what());
                resp->message = e.what();
                resp->success = false;
            }
            catch (...)
            {
                resp->message = "unknown error";
                resp->success = false;
            }

            return true;
        };
    auto service = nh.create_service<snapshotter::srv::TakeSnapshot>("take_snapshot", takeSnapshotCb);

    int numThreads = nh.get_parameter("num_threads").as_int();

    if (numThreads <= 0)
    {
        throw std::runtime_error("Parameter 'num_threads' needs to be >= 1");
    }

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), numThreads);
    executor.add_node(nh.get_node_base_interface());
    executor.spin();
    return 0;
}
