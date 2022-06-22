#include <memory>

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>


// Stag includes
#include "stag/Stag.h"
#include "stag_ros/structures.hpp"
#include <stag_ros/msg/s_tag_marker.hpp>
#include <stag_ros/msg/s_tag_marker_array.hpp>

namespace stag_ros {
class DummyNode : public rclcpp::Node {
 public:
  explicit DummyNode();
  ~DummyNode();

 private:
  // Callbacks
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg);

  // Functions
  void loadParameters();
  bool getBundleIndex(const int id, int &bundle_index, int &tag_index);
  bool getTagIndex(const int id, int &tag_index);

  // STag handle
  Stag *stag;
  int stag_library;
  int error_correction;

  // ROS Subcribers
  image_transport::Subscriber imageSub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoSub;

  // ROS Publishers
  image_transport::Publisher imageDebugPub;
  rclcpp::Publisher<stag_ros::msg::STagMarkerArray>::SharedPtr bundlePub;
  rclcpp::Publisher<stag_ros::msg::STagMarkerArray>::SharedPtr markersPub;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

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