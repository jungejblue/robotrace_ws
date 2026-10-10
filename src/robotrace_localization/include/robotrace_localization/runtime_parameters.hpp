#pragma once

#include <ros/ros.h>
#include <ros/names.h>

#include <stdexcept>
#include <string>

namespace robotrace_localization {

// Both seed publisher and subscriber use this absolute, shared parameter.
// A conflicting private seed_topic from an older YAML must fail at startup
// instead of leaving initialization waiting on a different topic.
inline std::string gnssSeedTopic(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& private_nh) {
  const std::string parameter = "/robotrace/gnss_seed_topic";
  std::string topic = "/robotrace/gnss_seed";
  if (nh.hasParam(parameter) && !nh.getParam(parameter, topic))
    throw std::invalid_argument(parameter + " must be a string");
  std::string error;
  if (topic.empty() || topic.front() != '/' || topic == "/" ||
      !ros::names::validate(topic, error))
    throw std::invalid_argument(parameter + " must be a valid absolute ROS topic");

  if (private_nh.hasParam("seed_topic")) {
    std::string legacy;
    if (!private_nh.getParam("seed_topic", legacy) || legacy != topic)
      throw std::invalid_argument(
          private_nh.resolveName("seed_topic") +
          " is obsolete and conflicts with " + parameter +
          "; remove the private key and configure the shared parameter");
    ROS_WARN_STREAM(private_nh.resolveName("seed_topic")
                    << " is obsolete; remove it and use " << parameter);
  }
  return topic;
}

}  // namespace robotrace_localization
