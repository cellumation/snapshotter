#include "ros/ros.h"
#include "Snapshotter.hpp"
#include "TopicFilter.hpp"
#include <snapshotter_2/TakeSnapshot.h>
#include <cm_lib_ros/ParamHelper.hpp>
#include <vector>
#include <string>

using namespace snapshotter;


BagCompression getCompression(cm::ros::ParamHelper& ph)
{
    const std::string compression = ph.getParam<std::string>("bag_compression");
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

    cm::ros::ParamHelper ph(nh);

    TopicFilter topicFilter({ph.getParam<std::vector<std::string>>("exclude_topics")});

    Snapshotter::Config cfg;
    cfg.maxMemoryBytes = ph.getParam<uint32_t>("max_memory_mb") * 1024 * 1024;
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

    BagCompression compression = getCompression(ph);
    boost::function<bool (snapshotter_2::TakeSnapshotRequest&,
                          snapshotter_2::TakeSnapshotResponse&)> takeSnapshotCb =
    [&snapshotter, &compression] (snapshotter_2::TakeSnapshotRequest& req,
                                  snapshotter_2::TakeSnapshotResponse& resp)
    {
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

    ros::MultiThreadedSpinner spinner(ph.getParam<uint32_t>("num_threads"));
    spinner.spin();
    return 0;
}