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
#include "ros/ros.h"
#include "Snapshotter.hpp"
#include "TopicFilter.hpp"
#include <snapshotter/TakeSnapshot.h>
#include <vector>
#include <string>
#include <mutex>

using namespace snapshotter;


BagCompression getCompression(ros::NodeHandle & nh)
{
    std::string compression;
    if(!nh.getParam("bag_compression", compression))
    {
        throw std::runtime_error("Parameter 'bag_compression' missing");
    }

    if(compression == "none")
        return BagCompression::NONE;
    if(compression == "slow")
        return BagCompression::SLOW;
    if(compression == "fast")
        return BagCompression::FAST;
    throw std::runtime_error("Parameter bag_compression has illegal value: '" + compression + "'");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "snapshotter_test");
    ros::NodeHandle nh("~");

    std::vector<std::string> excludeRegexes;
    if(!nh.getParam("exclude_topics", excludeRegexes))
    {
        throw std::runtime_error("Parameter 'exclude_topics' missing");
    }
    TopicFilter topicFilter({excludeRegexes});

    Snapshotter::Config cfg;

    int maxMemoryMb = 0;
    if(!nh.getParam("max_memory_mb", maxMemoryMb))
    {
        throw std::runtime_error("Parameter 'max_memory_mb' missing");
    }
    cfg.maxMemoryBytes = size_t(maxMemoryMb) * size_t(1024 * 1024);

    if(!nh.getParam("nice_on_write", cfg.niceOnWrite))
    {
        throw std::runtime_error("Parameter 'nice_on_write' missing");
    }

    Snapshotter snapshotter(nh, cfg);

    boost::function<void (const ros::TimerEvent&)> subscribeTopics =
    [&snapshotter, &topicFilter] (const ros::TimerEvent&)
    {
        ros::master::V_TopicInfo allTopics;
        if(ros::master::getTopics(allTopics))
        {
            for(const ros::master::TopicInfo& ti : allTopics)
            {
                if(!topicFilter.exclude(ti.name))
                {
                    snapshotter.subscribe(ti.name);
                }
            }
        }
    };
    //call once to immediately subscribe to all available topics
    subscribeTopics(ros::TimerEvent{});
    ros::Timer topicCheck = nh.createTimer(ros::Duration(2.0), subscribeTopics);

    BagCompression compression = getCompression(nh);

    std::mutex takeSnapshotServiceLock;

    boost::function<bool (snapshotter::TakeSnapshotRequest&,
                          snapshotter::TakeSnapshotResponse&)> takeSnapshotCb =
    [&] (snapshotter::TakeSnapshotRequest& req,
         snapshotter::TakeSnapshotResponse& resp)
    {
        std::unique_lock<std::mutex> lock(takeSnapshotServiceLock, std::try_to_lock);
        if(!lock.owns_lock()){
            // mutex wasn't locked. Handle it.
            resp.success = false;
            resp.message = "Already taking a snapshot.";
            return true;
        }

        try
        {
            snapshotter.writeBagFile(req.filename, compression);
            resp.success = true;
        }
        catch(const std::exception& e)
        {
            resp.message = e.what();
            resp.success = false;
        }
        catch(...)
        {
            resp.message = "unknown error";
            resp.success = false;
        }

        return true;
    };
    auto serv = nh.advertiseService("take_snapshot",takeSnapshotCb);

    int numThreads = 1;
    if(!nh.getParam("num_threads", numThreads))
    {
        throw std::runtime_error("Parameter 'num_threads' missing");
    }

    if(numThreads <= 0)
    {
        throw std::runtime_error("Parameter 'num_threads' needs to be >= 1");
    }

    ros::MultiThreadedSpinner spinner(numThreads);
    spinner.spin();
    return 0;
}