/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Cellumation GmbH
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
#include "ReductionRule.hpp"
#include <gtest/gtest.h>
#include <rclcpp/node.hpp>

using namespace snapshotter;

TEST(ReductionRulesParsing, ParsesAllRulesFromYaml)
{
    rclcpp::NodeOptions opts;
    opts.automatically_declare_parameters_from_overrides(true);
    rclcpp::Node nh("reduction_rules_tester", opts);

    const std::vector<ReductionRule> rules = parseReductionRules(nh);

    ASSERT_EQ(rules.size(), 3u);

    // Rule 0: drop_topic (from drop_topics list)
    EXPECT_TRUE(std::holds_alternative<DropRule>(rules[0]));
    EXPECT_TRUE(std::regex_match("/camera/image_raw", std::get<DropRule>(rules[0]).topicRegexp));
    EXPECT_FALSE(std::regex_match("/camera/image_raw/other", std::get<DropRule>(rules[0]).topicRegexp));

    // Rule 1: reduce_rate_to 2.5 Hz (first entry in reduce_rate_regexps)
    EXPECT_TRUE(std::holds_alternative<ReduceRule>(rules[1]));
    EXPECT_EQ(std::get<ReduceRule>(rules[1]).minInterval, rclcpp::Duration::from_seconds(1.0 / 2.5));
    EXPECT_TRUE(std::regex_match("/lidar/points", std::get<ReduceRule>(rules[1]).topicRegexp));
    EXPECT_TRUE(std::regex_match("/lidar/scan", std::get<ReduceRule>(rules[1]).topicRegexp));
    EXPECT_FALSE(std::regex_match("/lidar", std::get<ReduceRule>(rules[1]).topicRegexp));

    // Rule 2: reduce_rate_to 10 Hz (second entry in reduce_rate_regexps)
    EXPECT_TRUE(std::holds_alternative<ReduceRule>(rules[2]));
    EXPECT_EQ(std::get<ReduceRule>(rules[2]).minInterval, rclcpp::Duration::from_seconds(1.0 / 10.0));
    EXPECT_TRUE(std::regex_match("/imu/data", std::get<ReduceRule>(rules[2]).topicRegexp));
}

TEST(ReductionRulesParsing, EmptyWhenNoRulesDeclared)
{
    rclcpp::Node nh("reduction_rules_tester_empty");

    const std::vector<ReductionRule> rules = parseReductionRules(nh);

    EXPECT_TRUE(rules.empty());
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);

    // Pass the test params file so the node gets reduction_rules.* parameters.
    const char* extra[] = {argv[0], "--ros-args", "--params-file", TEST_PARAMS_FILE};
    int extra_argc = 4;
    rclcpp::init(extra_argc, const_cast<char**>(extra));

    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
