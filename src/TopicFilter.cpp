#include "TopicFilter.hpp"
#include <ros/console.h>
namespace snapshotter
{

TopicFilter::TopicFilter(const TopicFilterConfig& cfg)
{
    for(const std::string& regexp : cfg.excludeRegexps)
    {
        excludeRegexps.emplace_back(regexp);
        ROS_INFO_STREAM("Excluding topic: " << regexp);
    }
}

bool TopicFilter::exclude(const std::string& topic) const
{
    for(const boost::regex& r : excludeRegexps)
    {
        if(boost::regex_match(topic, r))
        {
            return true;
        }
    }
    return false;
}

}//end namespace