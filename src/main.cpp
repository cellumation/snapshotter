#include "ros/ros.h"
#include "snapshotter.hpp"
#include <snapshotter_2/TakeSnapshot.h>

using namespace snapshotter;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "snapshotter_test");
    ros::NodeHandle nh("~");

    Snapshotter::Config cfg;
    cfg.maxMemoryBytes = 1 * 1024 * 1024 * 1024; //TODO turn into parameter
    Snapshotter snapshotter(nh, cfg);

    boost::function<void (const ros::TimerEvent&)> subscribeTopics =
    [&snapshotter] (const ros::TimerEvent&)
    {
        ros::master::V_TopicInfo allTopics;
        if(ros::master::getTopics(allTopics))
        {
            for(const ros::master::TopicInfo& ti : allTopics)
            {
                snapshotter.subscribe(ti.name);
            }
        }
    };
    subscribeTopics(ros::TimerEvent{});
    ros::Timer topicCheck = nh.createTimer(ros::Duration(2.0), subscribeTopics);

    boost::function<bool (snapshotter_2::TakeSnapshotRequest&,
                          snapshotter_2::TakeSnapshotResponse&)> takeSnapshotCb =
    [&snapshotter] (snapshotter_2::TakeSnapshotRequest& req,
                    snapshotter_2::TakeSnapshotResponse& resp)
    {
        //TODO error handling!
        snapshotter.writeBagFile(req.filename);
        resp.success = true;
        return true;
    };
    auto serv = nh.advertiseService("take_snapshot",takeSnapshotCb);

    ros::MultiThreadedSpinner spinner(4); //TODO use num_threads parameter
    spinner.spin();
    return 0;

}