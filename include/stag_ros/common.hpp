/**
MIT License

Copyright (c) 2020 Brennan Cain (Unmanned Systems and Robotics Lab,
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
#pragma once

#include <vector>
#include <cv.hpp>

namespace stag_ros {

struct Common {
  static void solvePnpSingle(const std::vector<cv::Point2d> &img,
                       const std::vector<cv::Point3d> &world, cv::Mat &output,
                       const cv::Mat &cameraMatrix,
                       const cv::Mat &distortionMatrix) {
    if (img.empty() or world.empty()) return;
    cv::Mat rVec, rMat, tVec;
    //optimize for 5 planar points
    //possibly choose to reduce to the 4 for use with advanced algos
    cv::solvePnP(world, img, cameraMatrix, distortionMatrix, rVec, tVec);
    cv::Rodrigues(rVec, rMat);
    rMat.convertTo(output.colRange(0, 3), CV_64F);
    tVec.convertTo(output.col(3), CV_64F);
  }

  static void solvePnpBundle(const std::vector<cv::Point2d> &img,
                             const std::vector<cv::Point3d> &world, cv::Mat &output,
                             const cv::Mat &cameraMatrix,
                             const cv::Mat &distortionMatrix) {
    if (img.empty() or world.empty()) return;
    cv::Mat rVec, rMat, tVec;
    //optimize for many points
    cv::solvePnP(world, img, cameraMatrix, distortionMatrix, rVec, tVec);
    cv::Rodrigues(rVec, rMat);
    rMat.convertTo(output.colRange(0, 3), CV_64F);
    tVec.convertTo(output.col(3), CV_64F);
  }


};  // namespace stag_ros
}