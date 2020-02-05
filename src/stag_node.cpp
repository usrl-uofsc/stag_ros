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

#include "stag_ros/stag_node.h"

// ROS includes
#include "cv_bridge/cv_bridge.h"

// OpenCV includes
#include "opencv2/imgproc/imgproc.hpp"

// Stag marker handle
#include "Marker.h"

stag_node::stag_node(ros::NodeHandle& nh, image_transport::ImageTransport& imageT)
{
    // Set Subscribers
    imageSub = imageT.subscribe("/usb_cam/image_raw", 1, &stag_node::imageCallback, this);

    // Set Publishers
    // imageOut = imageT.advertise("/stag/image_raw", 1);

    // Initialize stag
    stag = new Stag(15, 7, false);
}

stag_node::~stag_node() {
    delete stag;
}

void stag_node::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat src = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat gray;
    cv::cvtColor(src, gray, CV_BGR2GRAY);

    // Process the image to find the marker
    stag->detectMarkers(gray);
    vector<Marker> markers = stag->getMarkerList();

    if (markers.size() > 0)
    {
        ROS_WARN("%d Markers detected", markers.size());

        for (int i = 0; i < markers.size(); i++)
        {
            ROS_INFO("Marker %d :", i);
            ROS_INFO("Marker corners:");
            ROS_INFO("Corner 1: %d, %d", markers[i].corners[1].x, markers[i].corners[1].y);
            ROS_INFO("Corner 2: %d, %d", markers[i].corners[2].x, markers[i].corners[2].y);
            ROS_INFO("Corner 3: %d, %d", markers[i].corners[3].x, markers[i].corners[3].y);
            ROS_INFO("Corner 4: %d, %d", markers[i].corners[4].x, markers[i].corners[4].y);
        }
    }
    else
        ROS_WARN("No markers detected");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stag_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport imageT(nh);

    stag_node stagN(nh, imageT);

    ros::spin();

    // ros::Rate r = ros::Rate(30);

    // while(ros::ok())
    // {
    //     ros::spinOnce();
    //     r.sleep();
    // }

    return 0;
}