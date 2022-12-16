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
 * @file move_base_action_wrapper.hpp
 * @author Nitesh Jha (Navigator), Tanuj Thakkar (Driver)
 * @brief This file contains the declaration of the MoveBaseActionWrapper class
 *
 */

#pragma once

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>

#include <memory>
#include <string>

namespace Artemis {

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

class MoveBaseActionWrapper {
 public:
  /**
   * @brief Construct a new Move Base Action Wrapper object
   *
   * @param action_name Name of the action server
   * @param spin_thread Whether to spin a thread for the action server
   */
  explicit MoveBaseActionWrapper(const std::string& action_name,
                                 const bool& spin_thread = true);

  /**
   * @brief Destroy the Move Base Action Wrapper object
   *
   */
  ~MoveBaseActionWrapper();

  /**
   * @brief Send a goal to the move base action server
   *
   * @param frame_id The frame id of the goal
   * @param goal The goal to be sent
   * @return true If the goal was sent successfully
   * @return false If the goal was not sent successfully
   */
  bool sendGoal(const std::string& frame_id,
                const geometry_msgs::PoseStamped& goal);

 private:
  std::string action_name_;
  std::shared_ptr<MoveBaseClient> move_base_client_;
};

}  // namespace Artemis
