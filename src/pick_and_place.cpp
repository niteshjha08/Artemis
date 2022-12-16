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
 * @file navigator.hpp
 * @author Nitesh Jha (Driver), Tanuj Thakkar (Navigator)
 * @brief This file contains the declaration of the PickAndPlace class
 *
 */

#include <pick_and_place.hpp>

namespace Artemis {

PickAndPlace::PickAndPlace(const ros::NodeHandle& node_handle)
    : node_handle_(node_handle) {
  shoulder_pub_ = node_handle_.advertise<std_msgs::Float64>(
      "/UR5_shoulder_lift_joint_controller/command", 10);
  elbow_pub_ = node_handle_.advertise<std_msgs::Float64>(
      "/UR5_shoulder_lift_joint_controller/command", 10);

  attach_client_ = node_handle_.serviceClient<gazebo_ros_link_attacher::Attach>(
      "/link_attacher_node/attach");
  detach_client_ = node_handle_.serviceClient<gazebo_ros_link_attacher::Attach>(
      "/link_attacher_node/detach");
}

PickAndPlace::~PickAndPlace() {}

void PickAndPlace::pickCargo(int task_id) {
  while (!attach_client_.waitForExistence(ros::Duration(1.0))) {
    ROS_INFO("Waiting for the attach service to come up");
  }
  ROS_INFO_STREAM("Attach service is up");

  ROS_ERROR_STREAM("calling attach service");
  double angle = -1.57;
  double increment = -0.1;
  double shoulder_pick_pose = -3.3;
  std_msgs::Float64 pubMsg;
  ros::Rate rate(1);
  while (angle > shoulder_pick_pose) {
    // shoulder_pub_.publish(angle);
    angle += increment;
    pubMsg.data = angle;
    shoulder_pub_.publish(pubMsg);
    rate.sleep();
  }
  pubMsg.data = -0.2;

  // Call attach service
  gazebo_ros_link_attacher::Attach attach_req;
  attach_req.request.model_name_1 = "husky";
  attach_req.request.link_name_1 = "ur_arm_wrist_3_link";
  attach_req.request.model_name_2 = "Box_Aruco_" + std::to_string(task_id);
  attach_req.request.link_name_2 = "link";
  if (attach_client_.call(attach_req)) {
    ROS_INFO_STREAM("Picked cargo successfully");
  } else {
    ROS_ERROR_STREAM("Failed to pick cargo.");
  }
}

void PickAndPlace::placeCargo(int task_id) {
  while (!attach_client_.waitForExistence(ros::Duration(1.0))) {
    ROS_INFO("Waiting for the attach service to come up");
  }

  ROS_INFO_STREAM("[Detaching link");

  // Call detach service
  gazebo_ros_link_attacher::Attach detach_req;
  detach_req.request.model_name_1 = "husky";
  detach_req.request.link_name_1 = "ur_arm_wrist_3_link";
  detach_req.request.model_name_2 = "Box_Aruco_" + std::to_string(task_id);
  detach_req.request.link_name_2 = "link";
  if (detach_client_.call(detach_req)) {
    ROS_INFO_STREAM("Placed cargo successfully");
  } else {
    ROS_ERROR_STREAM("Failed to call detach service");
  }

  double angle = -3.3;
  double increment = 0.1;
  double shoulder_place_pose = -1.57;
  std_msgs::Float64 pubMsg;
  ros::Rate rate(1);
  while (angle < shoulder_place_pose) {
    // shoulder_pub_.publish(angle);
    angle += increment;
    pubMsg.data = angle;
    shoulder_pub_.publish(pubMsg);
    rate.sleep();
  }
}
}  // namespace Artemis
