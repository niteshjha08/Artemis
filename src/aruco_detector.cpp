/******************************************************************************
 * MIT License

  Copyright (c) 2022 Nitesh Jha, Tanuj Thakkar

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
* *******************************************************************************
*/

/**
 * @copyright Copyright (c) 2022 Nitesh Jha, Tanuj Thakkar
 *
 * @file aruco_detector.cpp
 * @author Nitesh Jha (Navigator), Tanuj Thakkar (Driver)
 * @brief This file contains the definition of the class ArucoDetector
 *
 */

#include <aruco_detector.hpp>


namespace Artemis {

ArucoDetector::ArucoDetector(const ros::NodeHandle& node_handle, const std::string& task_id)
    : node_handle_(node_handle), task_id_(task_id) {}

ArucoDetector::~ArucoDetector() {}

void ArucoDetector::arucoDetectCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg) {
}

}  // namespace Artemis
