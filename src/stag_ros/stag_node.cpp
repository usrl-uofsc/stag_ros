#include "stag_ros/stag_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Project includes
#ifndef NDEBUG
#include "stag_ros/instrument.hpp"
#endif

// STag ROS
#include "stag_ros/utility.hpp"
#include "stag_ros/load_yaml_tags.hpp"
#include "stag_ros/common.hpp"

namespace stag_ros {
StagNode::StagNode() : Node("stag_node") {
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

  // Initialize the transform broadcaster
  this->tf_broadcaster =
      std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());

  // Load Parameters
  loadParameters();

  // Initialize Stag
  try {
    stag = new Stag(stag_library, error_correction, false);
  } catch (const std::invalid_argument &e) {
    std::cout << e.what() << std::endl;
    exit(-1);
  }

  // Set Subscribers
  imageSub = image_transport::create_subscription(
      this, image_topic,
      [this](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        this->imageCallback(msg);
      },
      is_compressed ? "compressed" : "raw", rmw_qos_profile_default);

  cameraInfoSub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, 10,
      [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        this->cameraInfoCallback(msg);
      });

  // Set Publishers
  if (debug_images) {
    imageDebugPub = image_transport::create_publisher(
        this, "stag_ros/image_markers", rmw_qos_profile_default);
  }
  bundlePub = this->create_publisher<stag_ros::msg::STagMarkerArray>(
      "stag_ros/bundles", 10);
  markersPub = this->create_publisher<stag_ros::msg::STagMarkerArray>(
      "stag_ros/markers", 10);

  // Initialize camera info
  got_camera_info = false;
  cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
  distortionMat = cv::Mat::zeros(1, 5, CV_64F);
  rectificationMat = cv::Mat::zeros(3, 3, CV_64F);
  projectionMat = cv::Mat::zeros(3, 4, CV_64F);
}

StagNode::~StagNode() { delete stag; }

void StagNode::loadParameters() {
  this->get_parameter("libraryHD", this->stag_library);
  this->get_parameter("errorCorrection", this->error_correction);
  this->get_parameter("raw_image_topic", this->image_topic);
  this->get_parameter("camera_info_topic", this->camera_info_topic);
  this->get_parameter("is_compressed", this->is_compressed);
  this->get_parameter("show_markers", this->debug_images);
  this->get_parameter("publish_tf", this->publish_tf);
  this->get_parameter("tag_tf_prefix", this->tag_tf_prefix);
  loadTagsBundles(this->shared_from_this(), "tags", "bundles", this->tags,
                  this->bundles);
}

bool StagNode::getTagIndex(int id, int &tag_index) {
  for (int i = 0; i < tags.size(); ++i) {
    if (tags[i].id == id) {
      tag_index = i;
      return true;
    }
  }
  return false;  // not found
}

bool StagNode::getBundleIndex(int id, int &bundle_index,
                               int &tag_index) {
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

void StagNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
#ifndef NDEBUG
  INSTRUMENT;
#endif
  if (got_camera_info) {
    cv::Mat gray;
    msgToGray(msg, gray);

    // Process the image to find the markers
    stag->detectMarkers(gray);
    std::vector<Marker> markers = stag->getMarkerList();

    // Publish debug image
    if (debug_images) {
      cv_bridge::CvImage rosMat;
      rosMat.header = msg->header;
      rosMat.encoding = "bgr8";
      rosMat.image = stag->drawMarkers();

      sensor_msgs::msg::Image rosImage;
      rosMat.toImageMsg(rosImage);

      imageDebugPub.publish(rosImage);
    }

    // For each marker in the list
    if (markers.size() > 0) {
      // Create markers msg
      std::vector<cv::Mat> tag_pose(tags.size());
      std::vector<cv::Mat> bundle_pose(bundles.size());
      std::vector<std::vector<cv::Point2d>> bundle_image(bundles.size());
      std::vector<std::vector<cv::Point3d>> bundle_world(bundles.size());

      for (int i = 0; i < markers.size(); i++) {
        // Create marker msg
        int tag_index, bundle_index;

        // if tag is a single tag, push back
        if (getTagIndex(markers[i].id, tag_index)) {
          std::vector<cv::Point2d> tag_image(5);
          std::vector<cv::Point3d> tag_world(5);

          tag_image[0] = markers[i].center;
          tag_world[0] = tags[tag_index].center;

          for (size_t ci = 0; ci < 4; ++ci) {
            tag_image[ci + 1] = markers[i].corners[ci];
            tag_world[ci + 1] = tags[tag_index].corners[ci];
          }

          cv::Mat marker_pose = cv::Mat::zeros(3, 4, CV_64F);
          Common::solvePnpSingle(tag_image, tag_world, marker_pose,
                                 cameraMatrix, distortionMat);
          tag_pose[tag_index] = marker_pose;

        } else if (getBundleIndex(markers[i].id, bundle_index, tag_index)) {
          bundle_image[bundle_index].push_back(markers[i].center);
          bundle_world[bundle_index].push_back(
              bundles[bundle_index].tags[tag_index].center);

          for (size_t ci = 0; ci < 4; ++ci) {
            bundle_image[bundle_index].push_back(markers[i].corners[ci]);
            bundle_world[bundle_index].push_back(
                bundles[bundle_index].tags[tag_index].corners[ci]);
          }
        }
      }

      for (size_t bi = 0; bi < bundles.size(); ++bi) {
        if (bundle_image[bi].size() > 0) {
          cv::Mat b_pose = cv::Mat::zeros(3, 4, CV_64F);
          Common::solvePnpBundle(bundle_image[bi], bundle_world[bi], b_pose,
                                 cameraMatrix, distortionMat);
          bundle_pose[bi] = b_pose;
        }
      }

      // Bundles
      std::vector<geometry_msgs::msg::Transform> bundle_transforms;
      std::vector<string> bundle_frame_ids;
      std::vector<int> bundle_ids;
      for (size_t bi = 0; bi < bundles.size(); ++bi) {
        if (bundle_pose[bi].empty()) continue;

        tf2::Matrix3x3 rotMat(
            bundle_pose[bi].at<double>(0, 0), bundle_pose[bi].at<double>(0, 1),
            bundle_pose[bi].at<double>(0, 2), bundle_pose[bi].at<double>(1, 0),
            bundle_pose[bi].at<double>(1, 1), bundle_pose[bi].at<double>(1, 2),
            bundle_pose[bi].at<double>(2, 0), bundle_pose[bi].at<double>(2, 1),
            bundle_pose[bi].at<double>(2, 2));
        tf2::Quaternion rotQ;
        rotMat.getRotation(rotQ);

        tf2::Vector3 tfVec(bundle_pose[bi].at<double>(0, 3),
                           bundle_pose[bi].at<double>(1, 3),
                           bundle_pose[bi].at<double>(2, 3));
        auto tmp_bundle_tf = tf2::Transform(rotQ, tfVec);
        geometry_msgs::msg::Transform bundle_tf;
        tf2::convert(tmp_bundle_tf, bundle_tf);

        bundle_transforms.push_back(bundle_tf);
        bundle_frame_ids.push_back(bundles[bi].frame_id);
        bundle_ids.push_back(-1);
      }

      if (bundle_ids.size() > 0)
        Common::publishTransform(bundle_transforms, tf_broadcaster, bundlePub,
                                 msg->header, tag_tf_prefix, bundle_frame_ids,
                                 bundle_ids, publish_tf);

      // Markers
      std::vector<geometry_msgs::msg::Transform> marker_transforms;
      std::vector<string> marker_frame_ids;
      std::vector<int> marker_ids;
      for (size_t ti = 0; ti < tags.size(); ++ti) {
        if (tag_pose[ti].empty()) continue;

        tf2::Matrix3x3 rotMat(
            tag_pose[ti].at<double>(0, 0), tag_pose[ti].at<double>(0, 1),
            tag_pose[ti].at<double>(0, 2), tag_pose[ti].at<double>(1, 0),
            tag_pose[ti].at<double>(1, 1), tag_pose[ti].at<double>(1, 2),
            tag_pose[ti].at<double>(2, 0), tag_pose[ti].at<double>(2, 1),
            tag_pose[ti].at<double>(2, 2));
        tf2::Quaternion rotQ;
        rotMat.getRotation(rotQ);

        tf2::Vector3 tfVec(tag_pose[ti].at<double>(0, 3),
                           tag_pose[ti].at<double>(1, 3),
                           tag_pose[ti].at<double>(2, 3));
        auto tmp_marker_tf = tf2::Transform(rotQ, tfVec);
        geometry_msgs::msg::Transform marker_tf;
        tf2::convert(tmp_marker_tf, marker_tf);

        marker_transforms.push_back(marker_tf);
        marker_frame_ids.push_back(tags[ti].frame_id);
        marker_ids.push_back(tags[ti].id);
      }

      if (marker_ids.size() > 0)
        Common::publishTransform(marker_transforms, tf_broadcaster, markersPub,
                                 msg->header, tag_tf_prefix, marker_frame_ids,
                                 marker_ids, publish_tf);

    } else {
      RCLCPP_WARN(this->get_logger(), "No markers detected");
    }
  }
}

void StagNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr &msg) {
  if (!got_camera_info) {
    // Get camera Matrix
    cameraMatrix.at<double>(0, 0) = msg->k[0];
    cameraMatrix.at<double>(0, 1) = msg->k[1];
    cameraMatrix.at<double>(0, 2) = msg->k[2];
    cameraMatrix.at<double>(1, 0) = msg->k[3];
    cameraMatrix.at<double>(1, 1) = msg->k[4];
    cameraMatrix.at<double>(1, 2) = msg->k[5];
    cameraMatrix.at<double>(2, 0) = msg->k[6];
    cameraMatrix.at<double>(2, 1) = msg->k[7];
    cameraMatrix.at<double>(2, 2) = msg->k[8];

    // Get distortion Matrix
    distortionMat = cv::Mat::zeros(1, msg->d.size(), CV_64F);
    for (size_t i = 0; i < msg->d.size(); i++)
      distortionMat.at<double>(0, i) = msg->d[i];

    // Get rectification Matrix
    rectificationMat.at<double>(0, 0) = msg->r[0];
    rectificationMat.at<double>(0, 1) = msg->r[1];
    rectificationMat.at<double>(0, 2) = msg->r[2];
    rectificationMat.at<double>(1, 0) = msg->r[3];
    rectificationMat.at<double>(1, 1) = msg->r[4];
    rectificationMat.at<double>(1, 2) = msg->r[5];
    rectificationMat.at<double>(2, 0) = msg->r[6];
    rectificationMat.at<double>(2, 1) = msg->r[7];
    rectificationMat.at<double>(2, 2) = msg->r[8];

    // Get projection Matrix
    projectionMat.at<double>(0, 0) = msg->p[0];
    projectionMat.at<double>(0, 1) = msg->p[1];
    projectionMat.at<double>(0, 2) = msg->p[2];
    projectionMat.at<double>(1, 0) = msg->p[3];
    projectionMat.at<double>(1, 1) = msg->p[4];
    projectionMat.at<double>(1, 2) = msg->p[5];
    projectionMat.at<double>(2, 0) = msg->p[6];
    projectionMat.at<double>(2, 1) = msg->p[7];
    projectionMat.at<double>(2, 2) = msg->p[8];
    projectionMat.at<double>(2, 0) = msg->p[9];
    projectionMat.at<double>(2, 1) = msg->p[10];
    projectionMat.at<double>(2, 2) = msg->p[11];

    got_camera_info = true;
  }
}
}  // namespace stag_ros

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stag_ros::StagNode>());
  rclcpp::shutdown();
  return 0;
}