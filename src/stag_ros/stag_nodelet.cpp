/**
MIT License

Copyright (c) 2020 Michail Kalaitzakis and Brennan Cain (Unmanned Systems and
Robotics Lab, University of South Carolina, USA)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// Project includes
#ifndef NDEBUG
#include "stag_ros/instrument.hpp"
#endif

#include "stag_ros/stag_nodelet.h"
#include "stag_ros/tag_json_loader.hpp"
#include "stag_ros/utility.hpp"

// Stag marker handle
#include "stag/Marker.h"

// ROS includes
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <future>
#include <swri_nodelet/class_list_macros.h>
#include <stag_ros/common.hpp>

SWRI_NODELET_EXPORT_CLASS(stag_ros, StagNodelet)

namespace stag_ros {

void StagNodelet::onInit() {
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle nh_lcl = getPrivateNodeHandle();
  image_transport::ImageTransport imageT(nh);

  // Load Parameters
  loadParameters(nh_lcl);

  // Set Subscribers
  imageSub = imageT.subscribe(
      imageTopic, 1, &StagNodelet::imageCallback, this,
      image_transport::TransportHints(isCompressed ? "compressed" : "raw"));

  cameraInfoSub =
      nh.subscribe(cameraInfoTopic, 1, &StagNodelet::cameraInfoCallback, this);

  // Set Publishers
  if (debugI) imageDebugPub = imageT.advertise("image_markers", 1);

  // Initialize Stag
  stag = new Stag(stagLib, errorC, false);

  // Initialize camera info
  gotCamInfo = false;
  cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
  distortionMat = cv::Mat::zeros(1, 5, CV_64F);
  rectificationMat = cv::Mat::zeros(3, 3, CV_64F);
  projectionMat = cv::Mat::zeros(3, 4, CV_64F);

}  // namespace stag_ros

StagNodelet::~StagNodelet() { delete stag; }

void StagNodelet::loadParameters(const ros::NodeHandle &nh) {
  // Create private nodeHandle to load parameters

  nh.param("libraryHD", stagLib, 15);
  nh.param("errorCorrection", errorC, 7);
  nh.param("raw_image_topic", imageTopic, std::string("image_raw"));
  nh.param("camera_info_topic", cameraInfoTopic, std::string("camera_info"));
  nh.param("is_compressed", isCompressed, false);
  nh.param("show_markers", debugI, false);
  nh.param("tag_tf_prefix", tag_tf_prefix, std::string("STag_"));

  std::string tagJson;
  nh.param("tag_config_json", tagJson, std::string());

  if (tagJson.compare("") == 0) {
    ROS_FATAL("No json specified");
    ros::shutdown();
  } else {
    tag_json_loader::load(tagJson, tags, bundles);
  }
}

bool StagNodelet::getTagIndex(const int id, int &tag_index) {
  for (int i = 0; i < tags.size(); ++i) {
    if (tags[i].id == id) {
      tag_index = i;
      return true;
    }
  }
  return false;  // not found
}

bool StagNodelet::getBundleIndex(const int id, int &bundle_index,
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

void StagNodelet::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
#ifndef NDEBUG
  INSTRUMENT;
#endif
  static tf::TransformBroadcaster br;
  if (gotCamInfo) {
    cv::Mat gray;
    msgToGray(msg, gray);

    // Process the image to find the markers
    stag->detectMarkers(gray);
    std::vector<Marker> markers = stag->getMarkerList();

    // Publish debug image
    if (debugI) {
      cv_bridge::CvImage rosMat;
      rosMat.header = msg->header;
      rosMat.encoding = "bgr8";
      rosMat.image = stag->drawMarkers();

      sensor_msgs::Image rosImage;
      rosMat.toImageMsg(rosImage);

      imageDebugPub.publish(rosImage);
    }

    // For each marker in the list
    if (markers.size() > 0) {
      // Create markers msg
      std::vector<cv::Mat> tag_pose(tags.size(), cv::Mat(3, 4, CV_64F));
      std::vector<cv::Mat> bundle_pose(bundles.size(), cv::Mat(3, 4, CV_64F));
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

          Common::solvePnpSingle(tag_image, tag_world, tag_pose[tag_index],
                                 cameraMatrix, distortionMat);

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
        if (bundle_image[bi].size() > 0)
          Common::solvePnpBundle(bundle_image[bi], bundle_world[bi],
                                 bundle_pose[bi], cameraMatrix, distortionMat);
      }

      for (size_t bi = 0; bi < bundles.size(); ++bi) {
        if (bundle_pose[bi].empty()) continue;

        tf::Matrix3x3 rotMat(
            bundle_pose[bi].at<double>(0, 0), bundle_pose[bi].at<double>(0, 1),
            bundle_pose[bi].at<double>(0, 2), bundle_pose[bi].at<double>(1, 0),
            bundle_pose[bi].at<double>(1, 1), bundle_pose[bi].at<double>(1, 2),
            bundle_pose[bi].at<double>(2, 0), bundle_pose[bi].at<double>(2, 1),
            bundle_pose[bi].at<double>(2, 2));
        tf::Quaternion rotQ;
        rotMat.getRotation(rotQ);

        tf::Vector3 tfVec(bundle_pose[bi].at<double>(0, 3),
                          bundle_pose[bi].at<double>(1, 3),
                          bundle_pose[bi].at<double>(2, 3));
        br.sendTransform(tf::StampedTransform(
            tf::Transform(rotQ, tfVec), msg->header.stamp, msg->header.frame_id,
            tag_tf_prefix + bundles[bi].frame_id));
      }
      for (size_t ti = 0; ti < tags.size(); ++ti) {
        if (tag_pose[ti].empty()) continue;

        tf::Matrix3x3 rotMat(
            tag_pose[ti].at<double>(0, 0), tag_pose[ti].at<double>(0, 1),
            tag_pose[ti].at<double>(0, 2), tag_pose[ti].at<double>(1, 0),
            tag_pose[ti].at<double>(1, 1), tag_pose[ti].at<double>(1, 2),
            tag_pose[ti].at<double>(2, 0), tag_pose[ti].at<double>(2, 1),
            tag_pose[ti].at<double>(2, 2));
        tf::Quaternion rotQ;
        rotMat.getRotation(rotQ);

        tf::Vector3 tfVec(tag_pose[ti].at<double>(0, 3),
                          tag_pose[ti].at<double>(1, 3),
                          tag_pose[ti].at<double>(2, 3));
        br.sendTransform(tf::StampedTransform(
            tf::Transform(rotQ, tfVec), msg->header.stamp, msg->header.frame_id,
            tag_tf_prefix + tags[ti].frame_id));
      }
    } else
      ROS_WARN("No markers detected");
  }
}

void StagNodelet::cameraInfoCallback(
    const sensor_msgs::CameraInfoConstPtr &msg) {
  if (!gotCamInfo) {
    // Get camera Matrix
    cameraMatrix.at<double>(0, 0) = msg->K[0];
    cameraMatrix.at<double>(0, 1) = msg->K[1];
    cameraMatrix.at<double>(0, 2) = msg->K[2];
    cameraMatrix.at<double>(1, 0) = msg->K[3];
    cameraMatrix.at<double>(1, 1) = msg->K[4];
    cameraMatrix.at<double>(1, 2) = msg->K[5];
    cameraMatrix.at<double>(2, 0) = msg->K[6];
    cameraMatrix.at<double>(2, 1) = msg->K[7];
    cameraMatrix.at<double>(2, 2) = msg->K[8];

    // Get distortion Matrix
    distortionMat.at<double>(0, 0) = msg->D[0];
    distortionMat.at<double>(0, 1) = msg->D[1];
    distortionMat.at<double>(0, 2) = msg->D[2];
    distortionMat.at<double>(0, 3) = msg->D[3];
    distortionMat.at<double>(0, 4) = msg->D[4];
    // Get rectification Matrix
    rectificationMat.at<double>(0, 0) = msg->R[0];
    rectificationMat.at<double>(0, 1) = msg->R[1];
    rectificationMat.at<double>(0, 2) = msg->R[2];
    rectificationMat.at<double>(1, 0) = msg->R[3];
    rectificationMat.at<double>(1, 1) = msg->R[4];
    rectificationMat.at<double>(1, 2) = msg->R[5];
    rectificationMat.at<double>(2, 0) = msg->R[6];
    rectificationMat.at<double>(2, 1) = msg->R[7];
    rectificationMat.at<double>(2, 2) = msg->R[8];
    // Get projection Matrix
    projectionMat.at<double>(0, 0) = msg->P[0];
    projectionMat.at<double>(0, 1) = msg->P[1];
    projectionMat.at<double>(0, 2) = msg->P[2];
    projectionMat.at<double>(1, 0) = msg->P[3];
    projectionMat.at<double>(1, 1) = msg->P[4];
    projectionMat.at<double>(1, 2) = msg->P[5];
    projectionMat.at<double>(2, 0) = msg->P[6];
    projectionMat.at<double>(2, 1) = msg->P[7];
    projectionMat.at<double>(2, 2) = msg->P[8];
    projectionMat.at<double>(2, 0) = msg->P[9];
    projectionMat.at<double>(2, 1) = msg->P[10];
    projectionMat.at<double>(2, 2) = msg->P[11];

    gotCamInfo = true;
  }
}
}  // namespace stag_ros