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
 * @file task_action_server.cpp
 * @author Nitesh Jha (Navigator), Tanuj Thakkar (Driver)
 * @brief This file contains the definition of the TaskActionServer class
 *
 */

#include <task_action_server.hpp>

namespace Artemis {

TaskActionServer::TaskActionServer(const ros::NodeHandle& node_handle,
                                   const std::string& action_name)
    : node_handle_(node_handle),
      action_name_(action_name),
      move_base_wrapper_(
          std::make_shared<MoveBaseActionWrapper>("move_base", true)),
      navigator_(move_base_wrapper_),
      task_action_server_(node_handle_, action_name,
                          boost::bind(&TaskActionServer::executeTask, this, _1),
                          false) {
  ROS_INFO_STREAM("TaskActionServer (" << action_name_
                                       << "): Starting task action server...");
  task_action_server_.start();
  ROS_INFO_STREAM("TaskActionServer (" << action_name_
                                       << "): Task action server started");
}

TaskActionServer::~TaskActionServer() {
  ROS_INFO_STREAM("TaskActionServer ("
                  << action_name_ << "): Shutting down task action server...");
  task_action_server_.shutdown();
  ROS_INFO_STREAM("TaskActionServer (" << action_name_
                                       << "): Task action server shut down");
}

void TaskActionServer::executeTask(const Artemis::TaskGoalConstPtr& task_goal) {
  ROS_INFO_STREAM("TaskActionServer ("
                  << action_name_ << "): Task request received. Executing...");

  ArucoDetector aruco_detector(node_handle_, task_goal->ID, "fiducial_transforms");
  geometry_msgs::PoseStamped task_pose;
  bool detected = false;
  int count = 0;

  // Search and detect the cargo
  while (task_action_server_.isActive() && ros::ok() && !detected) {
    // Check if the task action server has been preempted
    if (task_action_server_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO_STREAM("TaskActionServer ("
                      << action_name_ << "): Task action server preempted");
      task_action_server_.setPreempted();
      return;
    }

    if (!detected) {
      task_feedback_.process_status = "NAVIGATING";

      if (task_goal->staging_goals.poses.empty()) {
        ROS_ERROR_STREAM("TaskActionServer (" << action_name_
                                              << "): CARGO NOT FOUND");
        task_action_server_.setAborted();
        return;
      }

      geometry_msgs::PoseStamped staging_goal;
      staging_goal.header.frame_id = "map";
      staging_goal.header.stamp = ros::Time::now();
      staging_goal.pose = task_goal->staging_goals.poses[count];

      ROS_INFO_STREAM("TaskActionServer (" << action_name_
                                           << "): Navigating to staging goal");
      if (navigator_.navigate(staging_goal)) {
        count++;
        ROS_INFO_STREAM("TaskActionServer (" << action_name_
                                             << "): Reached staging goal");
        task_feedback_.process_status = "DETECTING";

        ros::Time detection_start_time = ros::Time::now();
        while (ros::Time::now() - detection_start_time < ros::Duration(5)) {
          if (aruco_detector.isDetected()) {
            ROS_INFO_STREAM("TaskActionServer (" << action_name_
                                                 << "): CARGO DETECTED");
            detected = true;
            task_pose = aruco_detector.getTaskPose();
            break;
          }
        }

      } else {
        ROS_ERROR_STREAM("TaskActionServer ("
                         << action_name_
                         << "): Failed to navigate to staging goal");
        task_action_server_.setAborted();
        return;
      }
    }

    ros::Duration(0.1).sleep();  // Sleep for 100 ms
  }
}

}  // namespace Artemis
