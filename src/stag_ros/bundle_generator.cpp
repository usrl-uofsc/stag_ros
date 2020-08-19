/**
MIT License

Copyright (c) 2020 Brennan Cain (Unmanned Systems and
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

#include "stag_ros/bundle_generator.h"
#include "stag_ros/tag_json_loader.hpp"
#include "stag_ros/utility.hpp"

// Stag marker handle
#include "stag/Marker.h"

// ROS includes
#include "tf/tf.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <stag_ros/common.hpp>

namespace stag_ros {

BundleGenerator::BundleGenerator(ros::NodeHandle &nh,
                                 image_transport::ImageTransport &imageT) {
  // Load Parameters
  loadParameters();

  // Set Subscribers
  imageSub = imageT.subscribe(
      imageTopic, 1, &BundleGenerator::imageCallback, this,
      image_transport::TransportHints(isCompressed ? "compressed" : "raw"));
  cameraInfoSub = nh.subscribe(cameraInfoTopic, 1,
                               &BundleGenerator::cameraInfoCallback, this);

  // Initialize Stag
  stag = new Stag(stagLib, errorC, false);

  // Initialize camera info
  gotCamInfo = false;
  cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
  distortionMat = cv::Mat::zeros(1, 5, CV_64F);
  rectificationMat = cv::Mat::zeros(3, 3, CV_64F);
  projectionMat = cv::Mat::zeros(3, 4, CV_64F);
  corner_obs.open("corner_obs.csv");
  corner_obs.precision(13);

  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
  linearSolver
      = new g2o::LinearSolverCholmod<g2o
  ::BlockSolver_6_3::PoseMatrixType>();
  g2o::BlockSolver_6_3 * solver_ptr
      = new g2o::BlockSolver_6_3(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);
}

BundleGenerator::~BundleGenerator() { delete stag; }

void BundleGenerator::loadParameters() {
  // Create private nodeHandle to load parameters
  ros::NodeHandle nh_lcl("~");

  nh_lcl.param("libraryHD", stagLib, 15);
  nh_lcl.param("errorCorrection", errorC, 7);
  nh_lcl.param("raw_image_topic", imageTopic, std::string("image_raw"));
  nh_lcl.param("camera_info_topic", cameraInfoTopic,
               std::string("camera_info"));
  nh_lcl.param("is_compressed", isCompressed, false);
  nh_lcl.param("show_markers", debugI, false);

  nh_lcl.param("tag_config_json", tagJson, std::string());

  if (tagJson.compare("") == 0) {
    ROS_FATAL("No json specified");
    ros::shutdown();
  }

  XmlRpc::XmlRpcValue ids, sizes;
  nh_lcl.param("tag_ids", ids, ids);
  nh_lcl.param("tag_sizes", sizes, sizes);
  for (int i = 0; i < ids.size(); ++i) {
    int id = ids[i];
    tag_ids.push_back(id);
    double size = sizes[i];
    tag_sizes.push_back(size);
    double hsize = size / 2.0;
    tag_points.insert({
        id, std::vector<cv::Point2d>(
                {cv::Point2d(0, 0), cv::Point2d(-hsize, hsize),
                 cv::Point2d(hsize, hsize), cv::Point2d(hsize, -hsize),
                 cv::Point2d(-hsize, -hsize)})});
    tag_id_to_index.insert({id,tag_points.size()-1});
  }
  frame_vertex_index = tag_ids.size();
}

void BundleGenerator::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
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

    // For each marker in the list
    if (markers.size() > 0) {
      for (int i = 0; i < markers.size(); i++) {
        if (tag_points.count(markers[i].id)) {
          std::vector<cv::Point2d> tag_image(5);
          std::vector<cv::Point3d> tag_world(5);

          tag_world = tag_points[markers[i].id];

          tag_image[0] = markers[i].center;
          for (size_t ci = 0; ci < 4; ++ci) {
            tag_image[ci + 1] = markers[i].corners[ci];
          }

          Common::solvePnpSingle(tag_image, tag_world, tag_pose[tag_index],
                                 cameraMatrix, distortionMat);
        }
      }
    } else
      ROS_WARN("No markers detected");
  }
}

void BundleGenerator::cameraInfoCallback(
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

int main(int argc, char **argv) {
  ros::init(argc, argv, "stag_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport imageT(nh);

  stag_ros::BundleGenerator stagN(nh, imageT);

  ros::spin();

  return 0;
}