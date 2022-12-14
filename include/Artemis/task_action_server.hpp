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
 * @file task_action_server.hpp
 * @author Nitesh Jha (Navigator), Tanuj Thakkar (Driver)
 * @brief This file contains the declaration of the TaskActionServer class
 *
 */

#pragma once

#include <Artemis/TaskAction.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include <memory>
#include <string>

#include <aruco_detector.hpp>
#include <move_base_action_wrapper.hpp>
#include <navigator.hpp>
#include <wms_service_client.hpp>


namespace Artemis {

/**
 * @brief This class is used to create an action server for the task
 *        action server
 */
class TaskActionServer {
 protected:
  ros::NodeHandle node_handle_;  // Node handle for the task action server
  std::string action_name_;      // Action name
  actionlib::SimpleActionServer<Artemis::TaskAction>
      task_action_server_;  // Task action server

  Artemis::TaskFeedback task_feedback_;  // Task feedback
  Artemis::TaskResult task_result_;      // Task result

 public:
  /**
   * @brief Constructor for the TaskActionServer class
   */
  TaskActionServer(const ros::NodeHandle& node_handle,
                   const std::string& action_name);

  /**
   * @brief Destructor for the TaskActionServer class
   */
  ~TaskActionServer();

  /**
   * @brief This function is used to execute the task action server
   * @param task_goal The goal of the task action server
   */
  void executeTask(const Artemis::TaskGoalConstPtr& task_goal);

 private:
  // WMSServiceClient wms_service_client_;  // WMSServiceClient object
  std::shared_ptr<MoveBaseActionWrapper>
      move_base_wrapper_;  // MoveBaseWrapper object
  Navigator navigator_;    // Navigator object
  // ArucoDetector aruco_detector_;  // ArucoDetector object
};

}  // namespace Artemis
