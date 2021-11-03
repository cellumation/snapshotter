#pragma once
#include <vector>
#include <string>
#include <boost/regex.hpp>

namespace snapshotter
{

class TopicFilter
{
public:
    struct TopicFilterConfig
    {
        std::vector<std::string> excludeRegexps;
    };

    TopicFilter(const TopicFilterConfig& cfg);

    /**returns true if the given @p topic should be excluded */
    bool exclude(const std::string& topic) const;


private:
    std::vector<boost::regex> excludeRegexps;

};

}