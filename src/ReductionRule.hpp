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
#pragma once
#include <rclcpp/node.hpp>
#include <regex>
#include <string>
#include <vector>

namespace snapshotter
{

struct ReductionRule
{
    enum class Action
    {
        dropTopic,
        reduceRateTo
    };

    std::regex topicRegexp;
    Action action;
    double rate; ///< samples/s, only used when action == reduceRateTo
};

/** Parses reduction_rules from node parameters.
 *  Expected parameter layout (using automatically_declare_parameters_from_overrides):
 *    reduction_drop_topics: ["<regex>", ...]
 *    reduction_reduce_topics: ["<regex>", ...]
 *    reduction_reduce_rates: [<double>, ...]   # parallel to reduction_reduce_topics
 *  Returns an empty vector when no rules are configured.
 */
inline std::vector<ReductionRule> parseReductionRules(rclcpp::Node& nh)
{
    std::vector<ReductionRule> rules;

    if (nh.has_parameter("reduction_drop_topics"))
    {
        const auto dropTopics = nh.get_parameter("reduction_drop_topics").as_string_array();
        for (const auto& topic : dropTopics)
        {
            ReductionRule rule;
            rule.action = ReductionRule::Action::dropTopic;
            rule.rate = 0.0;
            try
            {
                rule.topicRegexp = std::regex(topic);
            }
            catch (const std::regex_error& e)
            {
                throw std::runtime_error(std::string("Invalid regexp in reduction_drop_topics: ") + e.what());
            }
            rules.push_back(std::move(rule));
        }
    }

    if (nh.has_parameter("reduction_reduce_topics"))
    {
        if (!nh.has_parameter("reduction_reduce_rates"))
        {
            throw std::runtime_error("reduction_reduce_topics is set but reduction_reduce_rates is missing");
        }
        const auto regexps = nh.get_parameter("reduction_reduce_topics").as_string_array();
        const auto rates = nh.get_parameter("reduction_reduce_rates").as_double_array();
        if (regexps.size() != rates.size())
        {
            throw std::runtime_error("reduction_reduce_topics and reduction_reduce_rates must have the same length");
        }
        for (size_t i = 0; i < regexps.size(); ++i)
        {
            if (rates[i] <= 0.0)
            {
                throw std::runtime_error("reduction_reduce_rates[" + std::to_string(i) + "] must be > 0");
            }
            ReductionRule rule;
            rule.action = ReductionRule::Action::reduceRateTo;
            rule.rate = rates[i];
            try
            {
                rule.topicRegexp = std::regex(regexps[i]);
            }
            catch (const std::regex_error& e)
            {
                throw std::runtime_error("Invalid regexp in reduction_reduce_topics[" + std::to_string(i) +
                                         "]: " + e.what());
            }
            rules.push_back(std::move(rule));
        }
    }

    return rules;
}

} // namespace snapshotter
