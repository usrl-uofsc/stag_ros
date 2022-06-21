#include <memory>

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/image_encodings.hpp>



// Stag includes
#include "stag/Stag.h"
#include "stag_ros/structures.hpp"
#include "stag_ros/load_yaml_tags.hpp"

namespace stag_ros {
class DummyNode : public rclcpp::Node {
 public:
  explicit DummyNode();
  ~DummyNode();

 private:
  // Functions
  void loadParameters();
  bool getBundleIndex(const int id, int &bundle_index, int &tag_index);
  bool getTagIndex(const int id, int &tag_index);

  // STag handle
  Stag *stag;
  int stag_library;
  int error_correction;

  // Data
  cv::Mat cameraMatrix;
  cv::Mat distortionMat;
  cv::Mat rectificationMat;
  cv::Mat projectionMat;
  bool got_camera_info;
  bool debug_images;
  bool publish_tf;
  bool is_compressed;
  std::string image_topic;
  std::string camera_info_topic;
  std::string tag_tf_prefix;

  // Tag and bundle info
  std::string bundle_config_filepath;
  std::vector<Bundle> bundles;
  std::vector<Tag> tags;
};
}  // namespace stag_ros