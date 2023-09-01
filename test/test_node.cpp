
#include "Common.hpp"
#include "Snapshotter.hpp"
#include <filesystem>
#include <gtest/gtest.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#define LOG_PATH "/tmp/snapshotter_tests"

namespace fs = std::filesystem;
using namespace snapshotter;

rclcpp::Node* handle;
rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;

void spin(size_t n)
{
    for (size_t i = 0; i < n; i++)
    {
        executor->spin_all(std::chrono::milliseconds(10));
    }
}

void clearLogFolder()
{
    fs::remove_all(std::filesystem::path(LOG_PATH));
}

std::string getLogFileName()
{
    static int i = 0;
    const std::string file = std::string(LOG_PATH) + "/test_" + std::to_string(i) + "/";
    i++;
    return file;
}

bool subscribeWithTimeout(std::vector<std::string> topics, rclcpp::Duration timeout, Snapshotter& s)
{
    rclcpp::Time start = handle->now();

    while (!topics.empty())
    {
        for (auto it = topics.begin(); it != topics.end();)
        {
            if (s.subscribe(*it))
            {
                it = topics.erase(it);
            }
            else
            {
                it++;
            }
        }

        ::spin(10);

        if (handle->now() > start + timeout)
        {
            return false;
        };
    }

    return true;
}

struct DataPublisher
{
    DataPublisher(rclcpp::Node& nh) : nh(nh)
    {
        boolPub = nh.create_publisher<std_msgs::msg::Bool>("test_bool", 5);
        floatPub = nh.create_publisher<std_msgs::msg::Float32MultiArray>("test_float", 5);
        counterPub = nh.create_publisher<std_msgs::msg::Int32>("test_counter", 5);
    }

    /** runs the publisher until everything has been published */
    void run()
    {
        pubTimer = nh.create_timer(std::chrono::milliseconds(1), std::bind(&DataPublisher::publish, this));
        spin();
    }

    /** spin until everything has been published */
    void spin()
    {
        while (!done())
        {
            executor->spin_all(std::chrono::milliseconds(10));
        }

        // spin some more to make sure that everything has really been published.
        for (int i = 0; i < 10; i++)
        {
            executor->spin_all(std::chrono::milliseconds(10));
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void publish()
    {
        if (done())
        {
            pubTimer->cancel();
            return;
        }

        std_msgs::msg::Bool b;
        b.data = currentBool;
        currentBool = !currentBool;
        boolPub->publish(b);
        boolPubCount++;

        f.data.push_back(currentFloat);
        currentFloat += 1.0;
        floatPub->publish(f);
        floatPubCount++;

        std_msgs::msg::Int32 counter;
        counter.data = counterPubCount;
        counterPub->publish(counter);
        counterPubCount++;

        pubCount++;
    }

    bool done() { return pubCount >= 1000; }

    bool waitForSubscribers(const std::vector<rclcpp::PublisherBase::SharedPtr>& pubs, rclcpp::Duration timeout)
    {
        rclcpp::Time start = handle->now();

        bool allConnected = false;
        // spin until both subscribers are connected. This is important, otherwise
        // we might miss the first few messages (which would cause the test to fail)
        while (!allConnected)
        {
            allConnected = true;
            for (const auto& pub : pubs)
            {
                if (pub->get_subscription_count() == 0)
                {
                    allConnected = false;
                }
            }

            if (allConnected)
            {
                return true;
            }

            ::spin(10);

            if (handle->now() > start + timeout)
            {
                return false;
            };
        }
        return false;
    }

    /**if @p partialBag is true this will only check the pattern and not the count */
    void checkBoolMsgs(const std::string& bagFile, bool partialBag)
    {
        rosbag2_cpp::Reader reader;
        reader.open(bagFile);
        bool lastBool = false;
        size_t boolCount = 0;
        while (reader.has_next())
        {
            auto bag_message = reader.read_next();

            if (bag_message->topic_name != "test_bool")
            {
                continue;
            }

            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            rclcpp::Serialization<std_msgs::msg::Bool> serialization;
            std_msgs::msg::Bool msg;
            serialization.deserialize_message(&extracted_serialized_msg, &msg);

            // if we start somewhere inside the bag we need to know the initial value
            // to check the pattern
            if (partialBag && boolCount == 0)
            {
                lastBool = msg.data;
            }

            ASSERT_EQ(lastBool, msg.data);
            lastBool = !lastBool;
            boolCount++;
        }
        if (!partialBag)
        {
            ASSERT_EQ(boolCount, boolPubCount);
        }
    }

    /**if @p partialBag is true this will only check the pattern and not the count */
    void checkFloatMsgs(const std::string& bagFile, bool partialBag)
    {
        rosbag2_cpp::Reader reader;
        reader.open(bagFile);
        size_t lastFloatArraySize = 1;
        size_t floatCount = 0;
        while (reader.has_next())
        {
            auto bag_message = reader.read_next();

            if (bag_message->topic_name != "test_float")
            {
                continue;
            }

            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            rclcpp::Serialization<std_msgs::msg::Float32MultiArray> serialization;
            std_msgs::msg::Float32MultiArray msg;
            serialization.deserialize_message(&extracted_serialized_msg, &msg);

            if (partialBag && floatCount == 0)
            {
                lastFloatArraySize = msg.data.size();
            }

            ASSERT_EQ(lastFloatArraySize, msg.data.size());
            lastFloatArraySize++;
            float value = 1.0;
            for (float data : msg.data)
            {
                ASSERT_EQ(value, data);
                value += 1.0;
            }
            floatCount++;
        }
        if (!partialBag)
        {
            ASSERT_EQ(floatCount, floatPubCount);
        }
    }

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr boolPub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr floatPub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr counterPub;
    rclcpp::TimerBase::SharedPtr pubTimer;

    bool currentBool = false;
    float currentFloat = 1.0;
    size_t floatPubCount = 0;
    size_t boolPubCount = 0;
    size_t counterPubCount = 0;
    std_msgs::msg::Float32MultiArray f;
    size_t pubCount = 0;
    rclcpp::Node& nh;
};

TEST(TestSuite, SimpleTest)
{
    //     clearLogFolder();

    Snapshotter::Config cfg;
    cfg.maxMemoryBytes = 1 * 1024 * 1024 * 1024;
    Snapshotter snapshotter(*handle, cfg);

    // create the publisher after subscribing, otherwise we might miss the first messages
    DataPublisher pub(*handle);

    ASSERT_TRUE(::subscribeWithTimeout({"test_bool", "test_float"}, std::chrono::seconds(1), snapshotter));

    // spin until both subscribers are connected. This is important, otherwise
    // we might miss the first few messages (which would cause the test to fail)
    ASSERT_TRUE(pub.waitForSubscribers({pub.boolPub, pub.floatPub}, std::chrono::seconds(1)));

    // publish everything and wait for the publisher to finish
    pub.run();

    const std::string file = getLogFileName();
    snapshotter.writeBagFile(file, BagCompression::NONE);

    pub.checkBoolMsgs(file, false);
    pub.checkFloatMsgs(file, false);
}

TEST(TestSuite, DropAllMsgs)
{
    DataPublisher pub(*handle);
    Snapshotter::Config cfg;
    cfg.keepLatched = true;
    cfg.maxMemoryBytes = 0;
    Snapshotter snapshotter(*handle, cfg);

    ASSERT_TRUE(::subscribeWithTimeout({"test_bool", "test_float"}, std::chrono::seconds(1), snapshotter));

    // spin until both subscribers are connected. This is important, otherwise
    // we might miss the first few messages (which would cause the test to fail)
    ASSERT_TRUE(pub.waitForSubscribers({pub.boolPub, pub.floatPub}, std::chrono::seconds(1)));

    pub.run();

    const std::string file = getLogFileName();
    snapshotter.writeBagFile(file, BagCompression::NONE);

    rosbag2_cpp::Reader reader;
    reader.open(file);

    for (const auto& topicInfo : reader.get_metadata().topics_with_message_count)
    {
        ASSERT_EQ(topicInfo.message_count, 0);
    }
}

TEST(TestSuite, DropSomeMsgs)
{
    Snapshotter::Config cfg;
    cfg.maxMemoryBytes = 5000;
    cfg.keepLatched = false;
    Snapshotter snapshotter(*handle, cfg);

    // create the publisher after subscribing, otherwise we might miss the first messages
    DataPublisher pub(*handle);

    ASSERT_TRUE(::subscribeWithTimeout({"test_bool", "test_float"}, std::chrono::seconds(1), snapshotter));

    // spin until both subscribers are connected. This is important, otherwise
    // we might miss the first few messages (which would cause the test to fail)
    ASSERT_TRUE(pub.waitForSubscribers({pub.boolPub, pub.floatPub}, std::chrono::seconds(1)));

    // publish everything and wait for the publisher to finish
    pub.run();

    const std::string file = getLogFileName();
    snapshotter.writeBagFile(file, BagCompression::NONE);

    rosbag2_cpp::Reader reader;
    reader.open(file);

    for (const auto& topicInfo : reader.get_metadata().topics_with_message_count)
    {
        if (topicInfo.topic_metadata.name == pub.boolPub->get_topic_name())
        {
            ASSERT_EQ(topicInfo.message_count, pub.boolPubCount);
        }
        if (topicInfo.topic_metadata.name == pub.floatPub->get_topic_name())
        {
            ASSERT_EQ(topicInfo.message_count, pub.floatPubCount);
        }
    }

    pub.checkBoolMsgs(file, true);
    pub.checkFloatMsgs(file, true);
}

TEST(TestSuite, Latched)
{
    /**publish latched once. Spam log until we are sure that the latched would have been dropped.
     * check if latched is still present.*/

    Snapshotter::Config cfg;
    cfg.maxMemoryBytes = 3000;
    cfg.keepLatched = true;
    Snapshotter snapshotter(*handle, cfg);

    DataPublisher pub(*handle);
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr latchedPub =
        handle->create_publisher<std_msgs::msg::Int32>("test_latched", rclcpp::QoS(5).transient_local());

    ASSERT_TRUE(
        ::subscribeWithTimeout({"test_bool", "test_float", "test_latched"}, std::chrono::seconds(1), snapshotter));

    // spin until subscribers are connected.
    ASSERT_TRUE(pub.waitForSubscribers({pub.boolPub, pub.floatPub, latchedPub}, std::chrono::seconds(1)));

    std_msgs::msg::Int32 latchedMsg;
    latchedMsg.data = 42;
    latchedPub->publish(latchedMsg);

    // spam the snapshotter with data until we are sure that the latched message would have been dropped
    // if it would not have been latched
    pub.run();

    const std::string file = getLogFileName();
    snapshotter.writeBagFile(file, BagCompression::NONE);

    rosbag2_cpp::Reader reader;
    reader.open(file);
    bool msgFound = false;
    while (reader.has_next())
    {
        auto bag_message = reader.read_next();

        if (bag_message->topic_name != "test_latched")
        {
            continue;
        }

        rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
        rclcpp::Serialization<std_msgs::msg::Int32> serialization;
        std_msgs::msg::Int32 msg;
        serialization.deserialize_message(&extracted_serialized_msg, &msg);
        msgFound = true;
        ASSERT_EQ(42, msg.data);
    }
    ASSERT_TRUE(msgFound);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    rclcpp::Node nh("snapshotter_2_tester");
    handle = &nh;
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(nh.get_node_base_interface());

    // //create log folder
    const fs::path p(LOG_PATH);
    if (!fs::exists(p))
    {
        fs::create_directories(p);
    }

    clearLogFolder();

    int result = RUN_ALL_TESTS();

    clearLogFolder();

    return result;
}
