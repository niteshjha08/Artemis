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
 * @file task_action_client.hpp
 * @author Nitesh Jha (Navigator), Tanuj Thakkar (Driver)
 * @brief This file contains the declaration of the TaskActionClient class
 *
 */

#pragma once

#include <Artemis/TaskAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>

#include <string>

namespace Artemis {

/**
 * @brief This class is used to create an action client for the task
 *        action server
 */
class TaskActionClient {
 public:
  /**
   * @brief Constructor for the TaskActionClient class
   */
  explicit TaskActionClient(const std::string& action_name);

  /**
   * @brief Destructor for the TaskActionClient class
   */
  ~TaskActionClient();

  /**
   * @brief This function is used to send a goal to the task action server
   * @param goal The goal to be sent to the task action server
   * @return True if the goal was sent successfully, false otherwise
   */
  bool requestTaskAction(const Artemis::TaskGoal& goal);

  /**
   * @brief This function is used to wait for the result of the task action
   *        server
   */
  void waitForResult();

  /**
   * @brief This function is used to get the state of the task action server
   * @return The state of the task action server
   */
  actionlib::SimpleClientGoalState getState();

 private:
  actionlib::SimpleActionClient<Artemis::TaskAction> task_action_client_;
};

}  // namespace Artemis
