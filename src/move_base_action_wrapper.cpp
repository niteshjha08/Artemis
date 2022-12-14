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
 * @file move_base_action_wrapper.cpp
 * @author Nitesh Jha (Navigator), Tanuj Thakkar (Driver)
 * @brief This file contains the definition of the MoveBaseActionWrapper class
 *
 */

#include <move_base_action_wrapper.hpp>

namespace Artemis {

MoveBaseActionWrapper::MoveBaseActionWrapper(const std::string& action_name,
                                             const bool& spin_thread)
    : action_name_(action_name),
      move_base_client_(
          std::make_shared<MoveBaseClient>(action_name, spin_thread)) {
  while (!move_base_client_->waitForServer(ros::Duration(5.0))) {
    ROS_INFO_STREAM("MoveBaseActionWrapper ("
                    << action_name_
                    << "): Waiting for the move_base action server to come up");
  }
  ROS_INFO_STREAM("MoveBaseActionWrapper ("
                  << action_name_ << "): Connected to move_base action server");
}

MoveBaseActionWrapper::~MoveBaseActionWrapper() {
  ROS_INFO_STREAM("MoveBaseActionWrapper ("
                  << action_name_
                  << "): Shutting down the move_base action client");
}

bool MoveBaseActionWrapper::sendGoal(const std::string& frame_id,
                                     const geometry_msgs::PoseStamped& goal) {
  move_base_msgs::MoveBaseGoal move_base_goal;  // Create a goal
  move_base_goal.target_pose.header.frame_id = frame_id;
  move_base_goal.target_pose.header.stamp = ros::Time::now();
  move_base_goal.target_pose.pose = goal.pose;

  move_base_client_->sendGoal(move_base_goal);  // Send the goal

  move_base_client_->waitForResult();  // Wait for the result

  if (move_base_client_->getState() ==
      actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO_STREAM("MoveBaseActionWrapper (" << action_name_
                                              << "): REACHED THE GOAL!");
    return true;
  } else {
    ROS_INFO_STREAM(
        "MoveBaseActionWrapper ("
        << action_name_
        << "): The base failed to move to the goal for some reason");
    return false;
  }
}

}  // namespace Artemis
