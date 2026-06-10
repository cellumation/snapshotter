
#include "Common.hpp"
#include "ReductionRule.hpp"
#include "Snapshotter.hpp"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <future>
#include <gtest/gtest.h>
#include <limits>
#include <optional>
#include <rcl/service_introspection.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/service_utils.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <unistd.h>

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

/** Flushes the filesystem that @p filename is stored on
 */
void flushFilesystem(const std::string& filename)
{
    int fd = open(filename.c_str(), O_RDONLY);
    if (fd >= 0)
    {
        if (fsync(fd) != 0)
        {
            throw std::runtime_error(strerror(errno));
        }
        close(fd);
    }
    else
    {
        throw std::runtime_error(strerror(errno));
    }
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
    const std::string reducedFile = getLogFileName();
    std::promise<std::optional<BagWriteException>> writeDonePromise;
    auto writeDoneFuture = writeDonePromise.get_future();
    snapshotter.writeBagFile(file, std::optional<std::string>(reducedFile), BagCompression::NONE,
                             [&](const std::optional<BagWriteException>& maybeError,
                                 const rclcpp::Time& /*firstTimestamp*/,
                                 const rclcpp::Time& /*lastTimestamp*/) { writeDonePromise.set_value(maybeError); });
    ASSERT_EQ(writeDoneFuture.wait_for(std::chrono::seconds(20)), std::future_status::ready);
    ASSERT_FALSE(writeDoneFuture.get().has_value());
    flushFilesystem(file);
    pub.checkBoolMsgs(file, false);
    pub.checkFloatMsgs(file, false);
    pub.checkBoolMsgs(reducedFile, false);
    pub.checkFloatMsgs(reducedFile, false);
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
    const std::string reducedFile = getLogFileName();
    std::promise<std::optional<BagWriteException>> writeDonePromise;
    auto writeDoneFuture = writeDonePromise.get_future();
    snapshotter.writeBagFile(
        file, reducedFile, BagCompression::NONE,
        [&writeDonePromise](const std::optional<BagWriteException>& maybeError, const rclcpp::Time& /*firstTimestamp*/,
                            const rclcpp::Time& /*lastTimestamp*/) { writeDonePromise.set_value(maybeError); });
    ASSERT_EQ(writeDoneFuture.wait_for(std::chrono::seconds(20)), std::future_status::ready);
    ASSERT_FALSE(writeDoneFuture.get().has_value());

    flushFilesystem(file);
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
    const std::string reducedFile = getLogFileName();
    std::promise<std::optional<BagWriteException>> writeDonePromise;
    auto writeDoneFuture = writeDonePromise.get_future();
    snapshotter.writeBagFile(
        file, reducedFile, BagCompression::NONE,
        [&writeDonePromise](const std::optional<BagWriteException>& maybeError, const rclcpp::Time& /*firstTimestamp*/,
                            const rclcpp::Time& /*lastTimestamp*/) { writeDonePromise.set_value(maybeError); });
    ASSERT_EQ(writeDoneFuture.wait_for(std::chrono::seconds(20)), std::future_status::ready);
    ASSERT_FALSE(writeDoneFuture.get().has_value());

    flushFilesystem(file);
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
    const std::string reducedFile = getLogFileName();
    std::promise<std::optional<BagWriteException>> writeDonePromise;
    auto writeDoneFuture = writeDonePromise.get_future();
    snapshotter.writeBagFile(
        file, reducedFile, BagCompression::NONE,
        [&writeDonePromise](const std::optional<BagWriteException>& maybeError, const rclcpp::Time& /*firstTimestamp*/,
                            const rclcpp::Time& /*lastTimestamp*/) { writeDonePromise.set_value(maybeError); });
    ASSERT_EQ(writeDoneFuture.wait_for(std::chrono::seconds(20)), std::future_status::ready);
    ASSERT_FALSE(writeDoneFuture.get().has_value());

    flushFilesystem(file);
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

TEST(TestSuite, ServiceLogging)
{
    Snapshotter::Config cfg;
    cfg.maxMemoryBytes = 1 * 1024 * 1024 * 1024;
    Snapshotter snapshotter(*handle, cfg);

    const std::string serviceName = "/test_service_for_logging";
    const std::string serviceEventTopic = rosbag2_cpp::service_name_to_service_event_topic_name(serviceName);

    auto srvCallback = [](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        response->success = request->data;
        response->message = "ok";
    };

    auto service = handle->create_service<std_srvs::srv::SetBool>(serviceName, srvCallback);
    service->configure_introspection(handle->get_clock(), rclcpp::SystemDefaultsQoS(),
                                     RCL_SERVICE_INTROSPECTION_CONTENTS);

    auto client = handle->create_client<std_srvs::srv::SetBool>(serviceName);
    client->configure_introspection(handle->get_clock(), rclcpp::SystemDefaultsQoS(),
                                    RCL_SERVICE_INTROSPECTION_CONTENTS);

    // Wait for the service event topic publisher to appear, then subscribe
    {
        rclcpp::Time start = handle->now();
        bool subscribed = false;
        while (!subscribed && (handle->now() - start) < rclcpp::Duration(std::chrono::seconds(5)))
        {
            subscribed = snapshotter.subscribe(serviceEventTopic);
            ::spin(10);
        }
        ASSERT_TRUE(subscribed) << "Failed to subscribe to service event topic";
    }

    // Wait for subscriber to be connected
    ::spin(50);

    // Wait for service to be ready
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

    // Send a few requests
    for (int i = 0; i < 3; i++)
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        auto future = client->async_send_request(request);

        // Spin until the future completes
        rclcpp::Time start = handle->now();
        while (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready &&
               (handle->now() - start) < rclcpp::Duration(std::chrono::seconds(5)))
        {
            ::spin(10);
        }
        ASSERT_EQ(future.wait_for(std::chrono::milliseconds(0)), std::future_status::ready)
            << "Service call did not complete in time";
    }

    // Spin some more to make sure all service events have been received
    ::spin(100);

    const std::string file = getLogFileName();
    const std::string reducedFile = getLogFileName();
    std::promise<std::optional<BagWriteException>> writeDonePromise;
    auto writeDoneFuture = writeDonePromise.get_future();
    snapshotter.writeBagFile(
        file, reducedFile, BagCompression::NONE,
        [&writeDonePromise](const std::optional<BagWriteException>& maybeError, const rclcpp::Time& /*firstTimestamp*/,
                            const rclcpp::Time& /*lastTimestamp*/) { writeDonePromise.set_value(maybeError); });
    ASSERT_EQ(writeDoneFuture.wait_for(std::chrono::seconds(20)), std::future_status::ready);
    ASSERT_FALSE(writeDoneFuture.get().has_value());

    flushFilesystem(file);
    rosbag2_cpp::Reader reader;
    reader.open(file);

    size_t serviceEventCount = 0;
    bool foundServiceEventTopic = false;
    for (const auto& topicInfo : reader.get_metadata().topics_with_message_count)
    {
        if (topicInfo.topic_metadata.name == serviceEventTopic)
        {
            foundServiceEventTopic = true;
            serviceEventCount = topicInfo.message_count;
        }
    }
    ASSERT_TRUE(foundServiceEventTopic) << "Service event topic not found in bag";
    // 3 requests with introspection on both client and service side produces multiple events per call
    // (REQUEST_SENT, REQUEST_RECEIVED, RESPONSE_SENT, RESPONSE_RECEIVED)
    // At minimum we expect some events to be logged
    ASSERT_GT(serviceEventCount, 0u) << "No service event messages found in bag";
}

TEST(TestSuite, TimestampCorrectness)
{
    Snapshotter::Config cfg;
    cfg.maxMemoryBytes = 1 * 1024 * 1024 * 1024;
    Snapshotter snapshotter(*handle, cfg);

    // Record the start time
    rclcpp::Time startTime = handle->now();

    // Create the publisher after subscribing, otherwise we might miss the first messages
    DataPublisher pub(*handle);

    ASSERT_TRUE(::subscribeWithTimeout({"test_bool", "test_float"}, std::chrono::seconds(1), snapshotter));

    // spin until both subscribers are connected. This is important, otherwise
    // we might miss the first few messages (which would cause the test to fail)
    ASSERT_TRUE(pub.waitForSubscribers({pub.boolPub, pub.floatPub}, std::chrono::seconds(1)));

    // publish everything and wait for the publisher to finish
    pub.run();

    // Record the end time
    rclcpp::Time endTime = handle->now();

    const std::string file = getLogFileName();
    std::promise<std::optional<BagWriteException>> writeDonePromise;
    std::promise<rclcpp::Time> firstTimestampPromise;
    std::promise<rclcpp::Time> lastTimestampPromise;
    auto writeDoneFuture = writeDonePromise.get_future();
    auto firstTimestampFuture = firstTimestampPromise.get_future();
    auto lastTimestampFuture = lastTimestampPromise.get_future();

    const std::string reducedFile = getLogFileName();
    snapshotter.writeBagFile(file, std::optional<std::string>(reducedFile), BagCompression::NONE,
                             [&](const std::optional<BagWriteException>& error, const rclcpp::Time& firstTimestamp,
                                 const rclcpp::Time& lastTimestamp) {
                                 writeDonePromise.set_value(error);
                                 firstTimestampPromise.set_value(firstTimestamp);
                                 lastTimestampPromise.set_value(lastTimestamp);
                             });

    ASSERT_EQ(writeDoneFuture.wait_for(std::chrono::seconds(20)), std::future_status::ready);
    ASSERT_FALSE(writeDoneFuture.get().has_value());

    // Get the timestamps from the callback
    ASSERT_EQ(firstTimestampFuture.wait_for(std::chrono::seconds(1)), std::future_status::ready);
    ASSERT_EQ(lastTimestampFuture.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    rclcpp::Time firstTimestamp = firstTimestampFuture.get();
    rclcpp::Time lastTimestamp = lastTimestampFuture.get();

    // Verify that timestamps are reasonable
    ASSERT_GT(firstTimestamp.seconds(), 0.0) << "First timestamp should be positive";
    ASSERT_GT(lastTimestamp.seconds(), 0.0) << "Last timestamp should be positive";
    ASSERT_LE(firstTimestamp, lastTimestamp) << "First timestamp should be <= last timestamp";

    // Verify that timestamps are within the expected time range
    // Allow some tolerance for timing differences
    ASSERT_GE(firstTimestamp.seconds(), startTime.seconds() - 1.0) << "First timestamp should be close to start time";
    ASSERT_LE(lastTimestamp.seconds(), endTime.seconds() + 1.0) << "Last timestamp should be close to end time";

    // Verify timestamps by reading the bag file directly
    flushFilesystem(file);
    rosbag2_cpp::Reader reader;
    reader.open(file);

    double bagFirstTimeSeconds = std::numeric_limits<double>::max();
    double bagLastTimeSeconds = std::numeric_limits<double>::min();
    bool foundNonLatchedMessage = false;

    while (reader.has_next())
    {
        auto bag_message = reader.read_next();

        // Skip latched topics (they are written with artificial timestamps)
        if (bag_message->topic_name == "test_latched")
        {
            continue;
        }

        double msgTimeSeconds = bag_message->recv_timestamp * 1e-9; // Convert nanoseconds to seconds
        bagFirstTimeSeconds = std::min(bagFirstTimeSeconds, msgTimeSeconds);
        bagLastTimeSeconds = std::max(bagLastTimeSeconds, msgTimeSeconds);
        foundNonLatchedMessage = true;
    }

    ASSERT_TRUE(foundNonLatchedMessage) << "Should have found non-latched messages";

    // Compare callback timestamps with actual bag timestamps using seconds to avoid time source issues
    EXPECT_NEAR(firstTimestamp.seconds(), bagFirstTimeSeconds, 0.001)
        << "First timestamp from callback should match bag first timestamp";
    EXPECT_NEAR(lastTimestamp.seconds(), bagLastTimeSeconds, 0.001)
        << "Last timestamp from callback should match bag last timestamp";
}

TEST(TestSuite, ReducedBagDropTopic)
{
    Snapshotter::Config cfg;
    cfg.maxMemoryBytes = 1 * 1024 * 1024 * 1024;

    cfg.reductionRules.emplace_back(DropRule{std::regex("test_bool")});

    Snapshotter snapshotter(*handle, cfg);

    DataPublisher pub(*handle);
    ASSERT_TRUE(::subscribeWithTimeout({"test_bool", "test_float"}, std::chrono::seconds(1), snapshotter));
    ASSERT_TRUE(pub.waitForSubscribers({pub.boolPub, pub.floatPub}, std::chrono::seconds(1)));
    pub.run();

    const std::string file = getLogFileName();
    const std::string reducedFile = getLogFileName();
    std::promise<std::optional<BagWriteException>> writeDonePromise;
    auto writeDoneFuture = writeDonePromise.get_future();
    snapshotter.writeBagFile(
        file, reducedFile, BagCompression::NONE,
        [&writeDonePromise](const std::optional<BagWriteException>& maybeError, const rclcpp::Time& /*firstTimestamp*/,
                            const rclcpp::Time& /*lastTimestamp*/) { writeDonePromise.set_value(maybeError); });
    ASSERT_EQ(writeDoneFuture.wait_for(std::chrono::seconds(20)), std::future_status::ready);
    ASSERT_FALSE(writeDoneFuture.get().has_value());

    flushFilesystem(file);
    flushFilesystem(reducedFile);

    // main bag must contain test_bool
    pub.checkBoolMsgs(file, false);

    // reduced bag must NOT contain test_bool, but must still contain test_float
    {
        rosbag2_cpp::Reader reader;
        reader.open(reducedFile);
        for (const auto& topicInfo : reader.get_metadata().topics_with_message_count)
        {
            ASSERT_NE(topicInfo.topic_metadata.name, pub.boolPub->get_topic_name())
                << "dropped topic must not appear in reduced bag";
        }
    }
    pub.checkFloatMsgs(reducedFile, false);
}

TEST(TestSuite, ReducedBagReduceRate)
{
    Snapshotter::Config cfg;
    cfg.maxMemoryBytes = 1 * 1024 * 1024 * 1024;

    // Limit test_bool to 1 sample/s; the publisher fires at ~1 kHz so we expect a large reduction.
    cfg.reductionRules.emplace_back(ReduceRule{std::regex("test_bool"), rclcpp::Duration::from_seconds(1.0)});

    Snapshotter snapshotter(*handle, cfg);

    DataPublisher pub(*handle);
    ASSERT_TRUE(::subscribeWithTimeout({"test_bool", "test_float"}, std::chrono::seconds(1), snapshotter));
    ASSERT_TRUE(pub.waitForSubscribers({pub.boolPub, pub.floatPub}, std::chrono::seconds(1)));
    ::spin(20);
    pub.run();

    const std::string file = getLogFileName();
    const std::string reducedFile = getLogFileName();
    std::promise<std::optional<BagWriteException>> writeDonePromise;
    auto writeDoneFuture = writeDonePromise.get_future();
    snapshotter.writeBagFile(
        file, reducedFile, BagCompression::NONE,
        [&writeDonePromise](const std::optional<BagWriteException>& maybeError, const rclcpp::Time& /*firstTimestamp*/,
                            const rclcpp::Time& /*lastTimestamp*/) { writeDonePromise.set_value(maybeError); });
    ASSERT_EQ(writeDoneFuture.wait_for(std::chrono::seconds(20)), std::future_status::ready);
    ASSERT_FALSE(writeDoneFuture.get().has_value());

    flushFilesystem(file);
    flushFilesystem(reducedFile);

    // Count test_bool messages in both bags
    size_t fullBoolCount = 0;
    size_t reducedBoolCount = 0;

    {
        rosbag2_cpp::Reader reader;
        reader.open(file);
        for (const auto& topicInfo : reader.get_metadata().topics_with_message_count)
        {
            if (topicInfo.topic_metadata.name == "test_bool")
            {
                fullBoolCount = topicInfo.message_count;
            }
        }
    }
    {
        rosbag2_cpp::Reader reader;
        reader.open(reducedFile);
        for (const auto& topicInfo : reader.get_metadata().topics_with_message_count)
        {
            if (topicInfo.topic_metadata.name == "test_bool")
            {
                reducedBoolCount = topicInfo.message_count;
            }
        }
    }

    ASSERT_GT(fullBoolCount, 0u) << "main bag should contain bool messages";
    // reduced bag should have far fewer bool messages than the full bag
    ASSERT_LT(reducedBoolCount, fullBoolCount) << "reduced bag should have fewer bool messages than full bag";
    // test_float is not rate-limited and must be fully present in reduced bag
    pub.checkFloatMsgs(reducedFile, true);
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

    clearLogFolder();

    // //create log folder
    const fs::path p(LOG_PATH);
    if (!fs::exists(p))
    {
        fs::create_directories(p);
    }

    int result = RUN_ALL_TESTS();

    clearLogFolder();

    return result;
}
