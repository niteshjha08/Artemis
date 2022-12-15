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

ArucoDetector::ArucoDetector(const ros::NodeHandle& node_handle,
                             const int& task_id,
                             const std::string& detection_topic)
    : node_handle_(node_handle),
      task_id_(task_id),
      detection_topic_(detection_topic) {
  aruco_subscriber_ = node_handle_.subscribe(
      detection_topic_, 10, &ArucoDetector::arucoDetectCallback, this);
  ROS_INFO_STREAM("ArucoDetector ("
                  << detection_topic_
                  << "): Subscribed to topic: " << detection_topic_);
  ROS_INFO_STREAM("ArucoDetector ("
                  << detection_topic_
                  << "): Searching for aruco marker with ID: " << task_id_);
}

ArucoDetector::~ArucoDetector() {}

void ArucoDetector::arucoDetectCallback(
    const aruco_msgs::MarkerArray::ConstPtr& msg) {
  if (!detected_ && !msg->markers.size() == 0) {
    // ROS_INFO_STREAM("ArucoDetector (" << detection_topic_ << "): Detected "
    // << msg->markers.size() << " aruco markers");
    for (auto marker : msg->markers) {
      // ROS_INFO_STREAM("ArucoDetector (" << detection_topic_ << "): Marker ID:
      // " << marker.id);
      if (marker.id == task_id_) {
        detected_ = true;
        ROS_INFO_STREAM("ArucoDetector ("
                        << detection_topic_
                        << "): Detected aruco marker with ID: " << task_id_);
        task_marker_pose_ = marker;
        setTaskPose();
        break;
      }
    }
  }
}

geometry_msgs::PoseStamped ArucoDetector::getTaskPose() { return task_pose_; }

void ArucoDetector::setTaskPose() {
  // tf2_ros::Buffer tfBuffer;
  // tf2_ros::TransformListener tfListener(tfBuffer);

  // geometry_msgs::TransformStamped camera_to_base_link =
  // tfBuffer.lookupTransform(
  //     "base_link", "camera_realsense_gazebo", ros::Time(0),
  //     ros::Duration(10.0));

  task_pose_.header.frame_id = "map";
  task_pose_.header.stamp = ros::Time::now();
  task_pose_.pose.position.x = task_marker_pose_.pose.pose.position.x;
  task_pose_.pose.position.y = task_marker_pose_.pose.pose.position.y;
  task_pose_.pose.position.z = task_marker_pose_.pose.pose.position.z;
  task_pose_.pose.orientation.x = task_marker_pose_.pose.pose.orientation.x;
  task_pose_.pose.orientation.y = task_marker_pose_.pose.pose.orientation.y;
  task_pose_.pose.orientation.z = task_marker_pose_.pose.pose.orientation.z;
  task_pose_.pose.orientation.w = task_marker_pose_.pose.pose.orientation.w;

  // tf2::doTransform(task_pose_, task_pose_, camera_to_base_link);
  ROS_INFO_STREAM("ArucoDetector (" << detection_topic_
                                    << "): Task pose set in base_link frame");
}

bool ArucoDetector::isDetected() { return detected_; }

}  // namespace Artemis
