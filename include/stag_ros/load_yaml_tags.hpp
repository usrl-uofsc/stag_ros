#pragma once

#include <string>
#include <vector>
#include <tuple>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>

#include "structures.hpp"

namespace stag_ros {

inline std::vector<Tag> parseTags(const YAML::Node &tags_yaml,
                                  const std::string &frame_id = "") {
  std::vector<Tag> tags;
  for (auto &tag : tags_yaml) {
    Tag t;
    t.id = tag["id"].as<int>();
    t.frame_id = !frame_id.empty() ? frame_id : tag["frame"].as<std::string>();

    int idx = 0;
    for (auto &corner : tag["corners"]) {
      auto point = corner.as<std::vector<double>>();
      t.corners[idx] = cv::Point3d(point[0], point[1], point[2]);
      ++idx;
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
    auto frame_id = bundle["frame"].as<std::string>();
    b.frame_id = frame_id;
    b.tags = parseTags(bundle["tags"], frame_id);
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
  } else {
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
  } else {
    try {
      bundles = parseBundles(YAML::Load(bundle_str));
    } catch (std::exception &e) {
      RCLCPP_ERROR_STREAM(node.get_logger(),
                          "Error loading bundle descriptions: " << e.what());
    }
  }
}
}  // namespace stag_ros