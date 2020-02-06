/**
MIT License

Copyright (c) 2020 Michail Kalaitzakis (Unmanned Systems and Robotics Lab, 
University of South Carolina, USA)

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
#include "stag_ros/stag_node.h"
#include "stag_ros/StagMarker.h"
#include "stag_ros/StagMarkers.h"

// OpenCV includes
#include "opencv2/imgproc/imgproc.hpp"

// Stag marker handle
#include "Marker.h"

stag_node::stag_node(ros::NodeHandle& nh, image_transport::ImageTransport& imageT)
{
    // Load Parameters
    loadParameters();

    // Set Subscribers
    imageSub = imageT.subscribe(imageTopic, 1, &stag_node::imageCallback, this);
    cameraInfoSub = nh.subscribe(cameraInfoTopic, 1, &stag_node::cameraInfoCallback, this);

    // Set Publishers
    if (debugI)
        imageDebugPub = imageT.advertise("/stag/image_markers", 1);

    // Initialize Stag
    stag = new Stag(stagLib, errorC, false);

    // Initialize stuff
    gotCamInfo = false;
    cameraMatrix = cv::Mat::zeros(3, 3, CV_32FC2);
    distortionMat = cv::Mat::zeros(1, 5, CV_32FC2);
    rectificationMat = cv::Mat::zeros(3, 3, CV_32FC2);
    projectionMat = cv::Mat::zeros(3, 4, CV_32FC2);
}

stag_node::~stag_node() {
    delete stag;
}

void stag_node::loadParameters()
{
    // Create private nodeHandle to load parameters
    ros::NodeHandle nh_lcl("");

    nh_lcl.param("libraryHD", stagLib, 15);
    nh_lcl.param("errorCorrection", errorC, 7);
    nh_lcl.param("raw_image_topic", imageTopic, std::string("usb_cam/image_raw"));
    nh_lcl.param("camera_info_topic", cameraInfoTopic, std::string("usb_cam/camera_info"));
    nh_lcl.param("debug_images", debugI, false);
}

void stag_node::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (gotCamInfo)
    {
        // Read image from msg and convert it to grayscale
        cv::Mat src = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat gray;
        cv::cvtColor(src, gray, CV_BGR2GRAY);

        // Process the image to find the markers
        stag->detectMarkers(gray);
        vector<Marker> markers = stag->getMarkerList();

        // Publish debug image
        if (debugI)
        {
            cv_bridge::CvImage rosMat;
            rosMat.header.frame_id = cameraID;
            rosMat.encoding = "bgr8";
            rosMat.image = stag->drawMarkers();

            sensor_msgs::Image rosImage;
            rosMat.toImageMsg(rosImage);

            imageDebugPub.publish(rosImage);
        }

        // // For each marker in the list
        // if (markers.size() > 0)
        // {
        //     ROS_WARN("%d Markers detected", markers.size());

        //     for (int i = 0; i < markers.size(); i++)
        //     {
        //         ROS_INFO("Marker %d :", i);
        //         ROS_INFO("Marker id: %d", markers[i].id);
        //         ROS_INFO("Marker corners:");
        //         ROS_INFO("Corner 0: %f, %f", markers[i].corners[0].x, markers[i].corners[0].y);
        //         ROS_INFO("Corner 1: %f, %f", markers[i].corners[1].x, markers[i].corners[1].y);
        //         ROS_INFO("Corner 2: %f, %f", markers[i].corners[2].x, markers[i].corners[2].y);
        //         ROS_INFO("Corner 3: %f, %f", markers[i].corners[3].x, markers[i].corners[3].y);
        //     }
        // }
        // else
        //     ROS_WARN("No markers detected");
    }
}

void stag_node::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
    if (!gotCamInfo)
    {
        // Get frame ID
        cameraID = msg->header.frame_id;
        // Get camera Matrix
        cameraMatrix.at<double>(0,0) = msg->K[0]; cameraMatrix.at<double>(0,1) = msg->K[1]; cameraMatrix.at<double>(0,2) = msg->K[2];
        cameraMatrix.at<double>(1,0) = msg->K[3]; cameraMatrix.at<double>(1,1) = msg->K[4]; cameraMatrix.at<double>(1,2) = msg->K[5];
        cameraMatrix.at<double>(2,0) = msg->K[6]; cameraMatrix.at<double>(2,1) = msg->K[7]; cameraMatrix.at<double>(2,2) = msg->K[8];
        // Get distortion Matrix
        distortionMat.at<double>(0,0) = msg->D[0]; distortionMat.at<double>(0,1) = msg->D[1]; distortionMat.at<double>(0,2) = msg->D[2]; cameraMatrix.at<double>(1,0) = msg->D[3]; cameraMatrix.at<double>(1,1) = msg->D[4];
        // Get rectification Matrix
        rectificationMat.at<double>(0,0) = msg->R[0]; rectificationMat.at<double>(0,1) = msg->R[1]; rectificationMat.at<double>(0,2) = msg->R[2];
        rectificationMat.at<double>(1,0) = msg->R[3]; rectificationMat.at<double>(1,1) = msg->R[4]; rectificationMat.at<double>(1,2) = msg->R[5];
        rectificationMat.at<double>(2,0) = msg->R[6]; rectificationMat.at<double>(2,1) = msg->R[7]; rectificationMat.at<double>(2,2) = msg->R[8];
        // Get projection Matrix
        projectionMat.at<double>(0,0) = msg->P[0]; projectionMat.at<double>(0,1) = msg->P[1]; projectionMat.at<double>(0,2) = msg->P[2];  projectionMat.at<double>(1,0) = msg->P[3];
        projectionMat.at<double>(1,1) = msg->P[4]; projectionMat.at<double>(1,2) = msg->P[5]; projectionMat.at<double>(2,0) = msg->P[6];  projectionMat.at<double>(2,1) = msg->P[7];
        projectionMat.at<double>(2,2) = msg->P[8]; projectionMat.at<double>(2,0) = msg->P[9]; projectionMat.at<double>(2,1) = msg->P[10]; projectionMat.at<double>(2,2) = msg->P[11];

        gotCamInfo = true;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stag_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport imageT(nh);

    stag_node stagN(nh, imageT);

    ros::spin();

    return 0;
}