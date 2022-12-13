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
 * @file aruco_detector.hpp
 * @author Nitesh Jha (Navigator), Tanuj Thakkar (Driver)
 * @brief This file contains the declaration of the class ArucoDetector
 *
 */

#pragma once

#include <ros/ros.h>
#include <fiducial_msgs/FiducialTransform.h>
#include <fiducial_msgs/FiducialTransformArray.h>


namespace Artemis {

/**
 * @brief This class is used to detect the aruco markers
 */
class ArucoDetector {
  public:
    /**
     * @brief Constructor for ArucoDetector class
     * @param node_handle - NodeHandle for the node
     */
    ArucoDetector(const ros::NodeHandle& node_handle, const std::string& task_id);

    /**
     * @brief Destructor for ArucoDetector class
     */
    ~ArucoDetector();

    /**
     * @brief Callback function for aruco detection
     * @param msg - Message containing the pose of the aruco marker
     */
    void arucoDetectCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);

  private:
    const ros::Subscriber aruco_subscriber_;  // Subscriber for aruco detection
    const std::string task_id_;  // Task ID
    const fiducial_msgs::FiducialTransform task_pose_;  // Pose of the task
};

}  // namespace Artemis
