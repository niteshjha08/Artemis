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
 * @file task_action_client.cpp
 * @author Nitesh Jha (Navigator), Tanuj Thakkar (Driver)
 * @brief This file contains the defintion of the TaskActionClient class
 *
 */

#include <task_action_client.hpp>

namespace Artemis {

TaskActionClient::TaskActionClient(const std::string& action_name)
    : action_name_(action_name),
      task_action_client_(
          std::make_shared<actionlib::SimpleActionClient<Artemis::TaskAction>>(
              action_name_, true)) {
  while (!task_action_client_->waitForServer(ros::Duration(5.0))) {
    ROS_INFO_STREAM("TaskActionClient ("
                    << action_name_
                    << "): Waiting for the task action server...");
  }
  ROS_INFO_STREAM("TaskActionClient (" << action_name_
                                       << "): Task action server connected!");
}

TaskActionClient::~TaskActionClient() {
  ROS_INFO_STREAM("TaskActionClient (" << action_name_ << "): Shutting down!");
}

bool TaskActionClient::requestTaskAction(const Artemis::TaskGoal& task_goal) {
  ROS_INFO_STREAM("TaskActionClient (" << action_name_
                                       << "): Sending task request...");
  task_action_client_->sendGoal(task_goal);
}

void TaskActionClient::waitForResult() { task_action_client_->waitForResult(); }

actionlib::SimpleClientGoalState TaskActionClient::getState() {
  return task_action_client_->getState();
}

}  // namespace Artemis
