#include "tbai_config/YamlConfig.hpp"

#include <ros/ros.h>

namespace tbai {
namespace config {

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
YamlConfig::YamlConfig(const std::string &configPath, const char delim) : configPath_(configPath), delim_(delim) {}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
YamlConfig YamlConfig::fromRosParam(const std::string &pathParam, const char delim) {
    std::string configPath;
    ros::param::get(pathParam, configPath);
    return YamlConfig(configPath, delim);
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
std::vector<std::string> YamlConfig::split(const std::string &s) const {
    std::vector<std::string> result;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim_)) {
        result.push_back(item);
    }
    return result;
}

}  // namespace config
}  // namespace tbai