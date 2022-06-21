#include "stag_ros/dummy_node.hpp"

namespace stag_ros {
DummyNode::DummyNode() : Node("dummy_node") {
  this->declare_parameter<int>("libraryHD", 15);
  this->declare_parameter<int>("errorCorrection", 7);
  this->declare_parameter<std::string>("raw_image_topic", "image_raw");
  this->declare_parameter<std::string>("camera_info_topic", "camera_info");
  this->declare_parameter<bool>("is_compressed", false);
  this->declare_parameter<bool>("show_markers", false);
  this->declare_parameter<bool>("publish_tf", false);
  this->declare_parameter<std::string>("tag_tf_prefix", "STag_");
  this->declare_parameter<std::string>("tags", "");
  this->declare_parameter<std::string>("bundles", "");
}

DummyNode::~DummyNode() { delete stag; }

void DummyNode::loadParameters() {
  this->get_parameter("libraryHD", this->stag_library);
  this->get_parameter("errorCorrection", this->error_correction);
  this->get_parameter("raw_image_topic", this->image_topic);
  this->get_parameter("camera_info_topic", this->camera_info_topic);
  this->get_parameter("is_compressed", this->is_compressed);
  this->get_parameter("show_markers", this->debug_images);
  this->get_parameter("publish_tf", this->publish_tf);
  this->get_parameter("tag_tf_prefix", this->tag_tf_prefix);
  loadTagsBundles(this->shared_from_this(), "tags", "bundles", this->tags, this->bundles);
}

bool DummyNode::getTagIndex(const int id, int &tag_index) {
  for (int i = 0; i < tags.size(); ++i) {
    if (tags[i].id == id) {
      tag_index = i;
      return true;
    }
  }
  return false;  // not found
}

bool DummyNode::getBundleIndex(const int id, int &bundle_index, int &tag_index) {
  for (int bi = 0; bi < bundles.size(); ++bi) {
    for (int ti = 0; ti < bundles[bi].tags.size(); ++ti) {
      if (bundles[bi].tags[ti].id == id) {
        bundle_index = bi;
        tag_index = ti;
        return true;
      }
    }
  }
  return false;  // not found
}
}  // namespace stag_ros

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stag_ros::DummyNode>());
  rclcpp::shutdown();
  return 0;
}