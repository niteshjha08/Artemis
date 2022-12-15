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

#include <aruco_msgs/Marker.h>
#include <aruco_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <string>

namespace Artemis {

/**
 * @brief This class is used to detect the aruco markers
 */
class ArucoDetector {
 public:
  /**
   * @brief Constructor for ArucoDetector class
   * @param node_handle - NodeHandle for the node
   * @param task_id - Task ID
   */
  ArucoDetector(const ros::NodeHandle& node_handle, const int& task_id,
                const std::string& detection_topic);

  /**
   * @brief Destructor for ArucoDetector class
   */
  ~ArucoDetector();

  /**
   * @brief Callback function for aruco detection
   * @param msg - Message containing the pose of the aruco marker
   */
  void arucoDetectCallback(const aruco_msgs::MarkerArray::ConstPtr& msg);

  /**
   * @brief Get the Task Pose object
   *
   * @return geometry_msgs::Pose
   */
  geometry_msgs::PoseStamped getTaskPose();

  /**
   * @brief Set the Task Pose object
   *
   */
  void setTaskPose();

  /**
   * @brief Retrun the flag to check if the task is detected
   *
   */
  bool isDetected();

 private:
  ros::NodeHandle node_handle_;           // NodeHandle for the node
  const int task_id_;                     // Task ID
  const std::string detection_topic_;     // Topic for aruco detection
  ros::Subscriber aruco_subscriber_;      // Subscriber for aruco detection
  aruco_msgs::Marker task_marker_pose_;   // Marker of the task
  geometry_msgs::PoseStamped task_pose_;  // Pose of the task
  bool detected_ = false;  // Flag to check if the task is detected
};

}  // namespace Artemis
