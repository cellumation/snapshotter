
#include <gtest/gtest.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <experimental/filesystem> //FIXME for gcc8 remove experimental
#include "Snapshotter.hpp"
#include "Common.hpp"


#define LOG_PATH "/tmp/snapshotter_2_tests"

namespace fs = std::experimental::filesystem;
using namespace snapshotter;

ros::NodeHandle* handle;

void clearLogFolder()
{
    for(const auto& entry : fs::directory_iterator(LOG_PATH))
    {
        if(entry.status().type() == fs::file_type::regular &&
           entry.path().extension().string() == ".bag")
        {
            fs::remove(entry.path());
        }
    }
}

int getLogFileCount()
{
    int count = 0;
    for(const auto& entry : fs::directory_iterator(LOG_PATH))
    {
        if(entry.status().type() == fs::file_type::regular &&
           entry.path().extension().string() == ".bag")
        {
            count++;
        }
    }
    return count;
}

struct DataPublisher
{
    DataPublisher(ros::NodeHandle& nh)
    {
        boolPub = nh.advertise<std_msgs::Bool>("test_bool", 5);
        floatPub = nh.advertise<std_msgs::Float32MultiArray>("test_float", 5);
        pubTimer = nh.createTimer(ros::Duration(0.01), &DataPublisher::publish, this);
    }

    void publish(const ros::TimerEvent& )
    {
        if(pubCount == 1000)
        {
            return;
        }

        std_msgs::Bool b;
        b.data = currentBool;
        currentBool = !currentBool;
        boolPub.publish(b);

        f.data.push_back(currentFloat);
        currentFloat += 1.0;
        floatPub.publish(f);

        pubCount++;
    }

    bool done()
    {
        return pubCount == 1000;
    }

    void checkBag(const std::string& path)
    {
        rosbag::Bag bag(path);
        rosbag::View view(bag);

        bool lastBool = false;
        size_t lastFloatArraySize = 1;

        for(auto it = view.begin(); it != view.end(); it++)
        {
            const std::string topic = it->getTopic();
            if(topic == "/test_bool")
            {
                std_msgs::Bool::ConstPtr s = it->instantiate<std_msgs::Bool>();

                ASSERT_EQ(lastBool, s->data);
                lastBool = !lastBool;
            }
            else if(topic == "/test_float")
            {
                std_msgs::Float32MultiArray::ConstPtr f = it->instantiate<std_msgs::Float32MultiArray>();
                ASSERT_EQ(lastFloatArraySize, f->data.size());
                lastFloatArraySize++;
                float value = 1.0;
                for(float data : f->data)
                {
                    ASSERT_EQ(value, data);
                    value += 1.0;
                }
            }
        }
    }

    ros::Publisher boolPub;
    ros::Publisher floatPub;
    ros::Publisher strPub;
    ros::Timer pubTimer;

    bool currentBool = false;
    float currentFloat = 1.0;
    std_msgs::Float32MultiArray f;
    size_t pubCount = 0;
};


TEST(TestSuite, SimpleTest)
{
    clearLogFolder();

    DataPublisher pub(*handle);
    Snapshotter::Config cfg;
    cfg.maxMemoryBytes = 1 * 1024 * 1024 * 1024;
    Snapshotter snapshotter(*handle, cfg);
    snapshotter.subscribe("test_bool");
    snapshotter.subscribe("test_float");

    while(!pub.done())
    {
        ros::spinOnce();
    }

    const std::string file = std::string(LOG_PATH) + "/test1.bag";
    snapshotter.writeBagFile(file, BagCompression::NONE);
    pub.checkBag(file);
}



// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "snapshotter_2_tester");
    ros::NodeHandle nh;
    handle = &nh;


    // //create log folder
    const fs::path p(LOG_PATH);
    if(!fs::exists(p))
    {
        fs::create_directories(p);
    }

    clearLogFolder();

    int result = RUN_ALL_TESTS();

    clearLogFolder();

    return result;
}