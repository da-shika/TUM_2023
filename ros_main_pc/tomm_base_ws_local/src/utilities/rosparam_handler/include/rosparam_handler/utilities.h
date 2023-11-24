#ifndef ROSPARAM_HANDLER_UTILITIES_H
#define ROSPARAM_HANDLER_UTILITIES_H

#include <ros/ros.h>
#include <boost/algorithm/string.hpp>

namespace rosparam_handler {

/// \brief Sets the logger level according to a standardized parameter name 'verbosity'.
///
/// \param nodeHandle The ROS node handle to search for the parameter 'verbosity'.
inline void setLoggerLevel(const ros::NodeHandle& nodeHandle) {

    std::string verbosity;
    if (!nodeHandle.getParam("verbosity", verbosity)) {
        verbosity = "warning";
    }

    ros::console::Level level_ros;
    bool valid_verbosity{true};
    if (verbosity == "debug") {
        level_ros = ros::console::levels::Debug;
    } else if (verbosity == "info") {
        level_ros = ros::console::levels::Info;
    } else if (verbosity == "warning") {
        level_ros = ros::console::levels::Warn;
    } else if (verbosity == "error") {
        level_ros = ros::console::levels::Error;
    } else if (verbosity == "fatal") {
        level_ros = ros::console::levels::Fatal;
    } else {
        ROS_WARN_STREAM("Invalid verbosity level specified: " << verbosity << "! Falling back to INFO.");
        valid_verbosity = false;
    }
    if (valid_verbosity) {
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, level_ros)) {
            ros::console::notifyLoggerLevelsChanged();
            ROS_DEBUG_STREAM("Verbosity set to " << verbosity);
        }
    }
}

/// \brief Retrieve node name
///
/// @param privateNodeHandle The private ROS node handle (i.e.
/// ros::NodeHandle("~") ).
/// @return node name
inline std::string getNodeName(const ros::NodeHandle& privateNodeHandle) {
    std::string name_space = privateNodeHandle.getNamespace();
    std::stringstream tempString(name_space);
    std::string name;
    while (std::getline(tempString, name, '/')) {
        ;
    }
    return name;
}

inline std::string joinNamespaces(const std::vector<std::string>& ns_names)
{
  if(ns_names.empty())
    return "/";
  
  std::string ns_joined;
  for(const auto& ns : ns_names)
  {
    if(!ns.empty())
    {
      ns_joined += ns + "/";
    }
  }
  return ns_joined;
}

/// \brief Tests that parameter is not set on the parameter server
inline bool testConstParam(const std::string key) {
    if (ros::param::has(key)) {
        ROS_WARN_STREAM("Parameter " << key
                                     << "' was set on the parameter server eventhough it was defined to be constant.");
        return false;
    } else {
        return true;
    }
}

} // namespace rosparam_handler

#endif