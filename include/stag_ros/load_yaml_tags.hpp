#pragma once

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>

#include "structures.hpp"

namespace stag_ros {

inline std::vector<Tag> parseTags(const YAML::Node &tags_yaml) {
  std::vector<Tag> tags;
  for (auto &tag : tags_yaml) {
    Tag t;
    t.id = tag["id"].as<int>();  // tag id
    t.frame_id = tag["frame"].as<std::string>();

    for (std::size_t i = 0; i < tag["corners"].size(); ++i) {
      double x = tag["corners"][i][0].as<double>();
      double y = tag["corners"][i][1].as<double>();
      double z = tag["corners"][i][2].as<double>();
      t.corners[i] = cv::Point3d(x, y, z);
    }
    t.center = (t.corners[2] + t.corners[0]) / 2;
    // project the vector C1->C2 from C0 to get the fourth corner
    t.corners[3] = t.corners[0] + (t.corners[2] - t.corners[1]);

    tags.push_back(t);
  }

  return tags;
}

inline std::vector<Bundle> parseBundles(const YAML::Node &bundles_yaml) {
  std::vector<Bundle> bundles;
  for (auto &bundle : bundles_yaml) {
    Bundle b;
    b.frame_id = bundle["frame"].as<std::string>();
    b.tags = parseTags(bundle["tags"]);
    bundles.push_back(b);
  }

  return bundles;
}

inline void loadTagsBundles(const rclcpp::Node &node,
                            const std::string &tag_name,
                            const std::string &bundle_name,
                            std::vector<Tag> &tags,
                            std::vector<Bundle> &bundles) {
  std::string tag_str = node.get_parameter(tag_name).as_string();
  if (tag_str.empty()) {
    RCLCPP_WARN(node.get_logger(), "No tags specified");
  }
  else {
    try {
      tags = parseTags(YAML::Load(tag_str));
    } catch (std::exception &e) {
      RCLCPP_ERROR_STREAM(node.get_logger(),
                          "Error loading tag descriptions: " << e.what());
    }
  }
  std::string bundle_str = node.get_parameter(bundle_name).as_string();
  if (bundle_str.empty()) {
    RCLCPP_WARN(node.get_logger(), "No bundles specified");
  }
  else {
    try {
      bundles = parseBundles(YAML::Load(bundle_str));
    } catch (std::exception &e) {
      RCLCPP_ERROR_STREAM(node.get_logger(),
                          "Error loading bundle descriptions: " << e.what());
    }
  }
}
}  // namespace stag_ros