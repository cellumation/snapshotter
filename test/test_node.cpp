
#include <gtest/gtest.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <filesystem>
#include "Snapshotter.hpp"
#include "Common.hpp"


#define LOG_PATH "/tmp/snapshotter_tests"

namespace fs = std::filesystem;
using namespace snapshotter;

ros::NodeHandle* handle;


void spin(size_t n)
{
    for(size_t i = 0; i < n; i++)
    {
        ros::spinOnce();
    }
}

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

std::string getLogFileName()
{
    static int i = 0;
    const std::string file = std::string(LOG_PATH) + "/test_" + std::to_string(i) + ".bag";
    i++;
    return file;
}

struct DataPublisher
{
    DataPublisher(ros::NodeHandle& nh) : nh(nh)
    {
        boolPub = nh.advertise<std_msgs::Bool>("test_bool", 5);
        floatPub = nh.advertise<std_msgs::Float32MultiArray>("test_float", 5);
        counterPub = nh.advertise<std_msgs::Int32>("test_counter", 5);
    }

    /** runs the publisher until everything has been published */
    void run()
    {
        pubTimer = nh.createTimer(ros::Duration(0.001), &DataPublisher::publish, this);
        spin();
    }

    /** spin until everything has been published */
    void spin()
    {
        while(!done())
        {
            ros::spinOnce();
        }

        //spin some more to make sure that everything has really been published.
        for(int i = 0; i < 10; i++)
        {
            ros::spinOnce();
            ros::Duration(0.001).sleep();
        }
    }

    void publish(const ros::TimerEvent& )
    {
        if(done())
        {
            pubTimer.stop();
            return;
        }

        std_msgs::Bool b;
        b.data = currentBool;
        currentBool = !currentBool;
        boolPub.publish(b);
        boolPubCount++;

        f.data.push_back(currentFloat);
        currentFloat += 1.0;
        floatPub.publish(f);
        floatPubCount++;

        std_msgs::Int32 counter;
        counter.data = counterPubCount;
        counterPub.publish(counter);
        counterPubCount++;

        pubCount++;
    }

    bool done()
    {
        return pubCount >= 1000;
    }

    /**if @p partialBag is true this will only check the pattern and not the count */
    void checkBoolMsgs(const std::string& bagFile, bool partialBag)
    {
        rosbag::Bag bag(bagFile);
        rosbag::View view(bag);
        bool lastBool = false;
        size_t boolCount = 0;
        for(auto it = view.begin(); it != view.end(); it++)
        {
            const std::string topic = it->getTopic();
            if(topic == "/test_bool")
            {
                std_msgs::Bool::ConstPtr s = it->instantiate<std_msgs::Bool>();

                //if we start somewhere inside the bag we need to know the initial value
                //to check the pattern
                if(partialBag && boolCount == 0)
                {
                    lastBool = s->data;
                }

                ASSERT_EQ(lastBool, s->data);
                lastBool = !lastBool;
                boolCount++;
            }
        }
        if(!partialBag)
        {
            ASSERT_EQ(boolCount, boolPubCount);
        }
    }

    /**if @p partialBag is true this will only check the pattern and not the count */
    void checkFloatMsgs(const std::string& bagFile, bool partialBag)
    {
        rosbag::Bag bag(bagFile);
        rosbag::View view(bag);
        size_t lastFloatArraySize = 1;
        size_t floatCount = 0;

        for(auto it = view.begin(); it != view.end(); it++)
        {
            const std::string topic = it->getTopic();
            if(topic == "/test_float")
            {
                std_msgs::Float32MultiArray::ConstPtr f = it->instantiate<std_msgs::Float32MultiArray>();

                if(partialBag && floatCount == 0)
                {
                    lastFloatArraySize = f->data.size();
                }

                ASSERT_EQ(lastFloatArraySize, f->data.size());
                lastFloatArraySize++;
                float value = 1.0;
                for(float data : f->data)
                {
                    ASSERT_EQ(value, data);
                    value += 1.0;
                }
                floatCount++;
            }
        }
        if(!partialBag)
        {
            ASSERT_EQ(floatCount, floatPubCount);
        }
    }

    ros::Publisher boolPub;
    ros::Publisher floatPub;
    ros::Publisher counterPub;
    ros::Timer pubTimer;

    bool currentBool = false;
    float currentFloat = 1.0;
    size_t floatPubCount = 0;
    size_t boolPubCount = 0;
    size_t counterPubCount = 0;
    std_msgs::Float32MultiArray f;
    size_t pubCount = 0;
    ros::NodeHandle& nh;
};


TEST(TestSuite, SimpleTest)
{
    clearLogFolder();

    Snapshotter::Config cfg;
    cfg.maxMemoryBytes = 1 * 1024 * 1024 * 1024;
    Snapshotter snapshotter(*handle, cfg);
    snapshotter.subscribe("test_bool");
    snapshotter.subscribe("test_float");

    //create the publisher after subscribing, otherwise we might miss the first messages
    DataPublisher pub(*handle);

    //spin until both subscribers are connected. This is important, otherwise
    //we might miss the first few messages (which would cause the test to fail)
    while(pub.boolPub.getNumSubscribers() == 0 || pub.floatPub.getNumSubscribers() == 0)
    {
        spin(10);
    }

    //publish everything and wait for the publisher to finish
    pub.run();

    const std::string file = getLogFileName();
    snapshotter.writeBagFile(file, BagCompression::NONE);


    pub.checkBoolMsgs(file, false);
    pub.checkFloatMsgs(file, false);


    clearLogFolder();
}


TEST(TestSuite, DropAllMsgs)
{
    clearLogFolder();

    DataPublisher pub(*handle);
    Snapshotter::Config cfg;
    cfg.keepLatched = true;
    cfg.maxMemoryBytes = 0;
    Snapshotter snapshotter(*handle, cfg);
    snapshotter.subscribe("test_bool");
    snapshotter.subscribe("test_float");

    //spin until both subscribers are connected. This is important, otherwise
    //we might miss the first few messages (which would cause the test to fail)
    while(pub.boolPub.getNumSubscribers() == 0 || pub.floatPub.getNumSubscribers() == 0)
    {
        spin(10);
    }

    pub.run();


    const std::string file = getLogFileName();
    snapshotter.writeBagFile(file, BagCompression::NONE);
    rosbag::Bag bag(file);
    rosbag::View view(bag);
    ASSERT_EQ(view.size(), 0);

    clearLogFolder();
}


TEST(TestSuite, DropSomeMsgs)
{
    clearLogFolder();

    Snapshotter::Config cfg;
    cfg.maxMemoryBytes = 5000;
    cfg.keepLatched = false;
    Snapshotter snapshotter(*handle, cfg);
    snapshotter.subscribe("test_bool");
    snapshotter.subscribe("test_float");

    //create the publisher after subscribing, otherwise we might miss the first messages
    DataPublisher pub(*handle);

    //spin until both subscribers are connected. This is important, otherwise
    //we might miss the first few messages (which would cause the test to fail)
    while(pub.boolPub.getNumSubscribers() == 0 || pub.floatPub.getNumSubscribers() == 0)
    {
        spin(10);
    }

    //publish everything and wait for the publisher to finish
    pub.run();

    const std::string file = getLogFileName();
    snapshotter.writeBagFile(file, BagCompression::NONE);

    rosbag::Bag bag(file);
    rosbag::View view(bag);
    ASSERT_NE(view.size(), pub.boolPubCount + pub.floatPubCount);


    pub.checkBoolMsgs(file, true);
    pub.checkFloatMsgs(file, true);


    clearLogFolder();
}


TEST(TestSuite, Latched)
{
    /**publish latched once. Spam log until we are sure that the latched would have been dropped.
     * check if latched is still present.*/
    clearLogFolder();

    Snapshotter::Config cfg;
    cfg.maxMemoryBytes = 3000;
    cfg.keepLatched = true;
    Snapshotter snapshotter(*handle, cfg);
    snapshotter.subscribe("test_bool");
    snapshotter.subscribe("test_float");
    snapshotter.subscribe("test_latched");

    DataPublisher pub(*handle);
    ros::Publisher latchedPub = handle->advertise<std_msgs::Int32>("test_latched", 5, true);

    //spin until subscribers are connected.
    while(pub.boolPub.getNumSubscribers() == 0 || pub.floatPub.getNumSubscribers() == 0 ||
          latchedPub.getNumSubscribers() == 0)
    {
        spin(10);
    }

    std_msgs::Int32 latchedMsg;
    latchedMsg.data = 42;
    latchedPub.publish(latchedMsg);

    //spam the snapshotter with data until we are sure that the latched message would have been dropped
    //if it would not have been latched
    pub.run();

    const std::string file = getLogFileName();
    snapshotter.writeBagFile(file, BagCompression::NONE);

    rosbag::Bag bag(file);
    rosbag::View view(bag);
    bool msgFound = false;
    for(auto it = view.begin(); it != view.end(); it++)
    {
        const std::string topic = it->getTopic();
        if(topic == "/test_latched")
        {
            msgFound = true;
            std_msgs::Int32::ConstPtr s = it->instantiate<std_msgs::Int32>();
            ASSERT_EQ(42, s->data);
        }
    }
    ASSERT_TRUE(msgFound);
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