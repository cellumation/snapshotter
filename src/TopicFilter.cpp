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

#include "TopicFilter.hpp"
#include <rclcpp/logging.hpp>
#include <regex>

namespace snapshotter
{
TopicFilter::TopicFilter(const std::vector<std::string>& excludeRegexps_,
                         const std::vector<std::string>& includeRegexps_)
{
    auto log = rclcpp::get_logger("snapshotter");
    for (const std::string& regexp : excludeRegexps_)
    {
        excludeRegexps.emplace_back(regexp);
        RCLCPP_INFO_STREAM(log, "Excluding topic: " << regexp);
    }
    for (const std::string& regexp : includeRegexps_)
    {
        includeRegexps.emplace_back(regexp);
        RCLCPP_INFO_STREAM(log, "Including topic: " << regexp);
    }
}

bool TopicFilter::exclude(const std::string& topic) const
{
    for (const std::regex& er : excludeRegexps)
    {
        if (std::regex_match(topic, er))
        {
            for (const std::regex& ir : includeRegexps)
            {
                if (std::regex_match(topic, ir))
                {
                    return false;
                }
            }
            return true;
        }
    }
    return false;
}
} // namespace snapshotter
