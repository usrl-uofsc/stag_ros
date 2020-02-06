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

#ifndef STAG_NODE_H
#define STAG_NODE_H

// ROS includes
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "camera_info_manager/camera_info_manager.h"

// ROS msgs
#include "sensor_msgs/image_encodings.h"

// Stag includes
#include "../../src/Stag.h"

class stag_node {
public:
    stag_node(ros::NodeHandle& nh, image_transport::ImageTransport& imageT);
    ~stag_node();

private:
    // ROS Subcribers
    image_transport::Subscriber imageSub;
    ros::Subscriber cameraInfoSub;

    // ROS Publishers
    ros::Publisher markersPub;
    image_transport::Publisher imageDebugPub;

    // Callbacks
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

    // STag handle
    Stag* stag;

    // Functions
    void loadParameters();

    // Data
    cv::Mat cameraMatrix;
    cv::Mat distortionMat;
    cv::Mat rectificationMat;
    cv::Mat projectionMat;
    std::string cameraID;
    bool gotCamInfo;
    bool debugI;
    int stagLib;
    int errorC;
    std::string imageTopic;
    std::string cameraInfoTopic;
};

#endif // STAG_NODE_H