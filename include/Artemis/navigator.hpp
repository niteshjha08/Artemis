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
 * @author Nitesh Jha (Navigator), Tanuj Thakkar (Driver)
 * @brief This file contains the declaration of the Navigator class
 *
 */

#pragma once

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>

#include <memory>
#include <move_base_action_wrapper.hpp>

namespace Artemis {

/**
 * @brief This class is responsible for navigating the robot to a given goal
 */
class Navigator {
 public:
  /**
   * @brief Construct a new Navigator object
   *
   * @param move_base_client The move base action client
   */
  explicit Navigator(
      const std::shared_ptr<MoveBaseActionWrapper>& move_base_client);

  /**
   * @brief Destroy the Navigator object
   *
   */
  ~Navigator();

  /**
   * @brief Navigate the robot to a given goal
   *
   * @param goal The goal to navigate to
   * @return true If the goal was reached
   * @return false If the goal was not reached
   */
  bool navigate(const geometry_msgs::PoseStamped& goal);

 private:
  std::shared_ptr<MoveBaseActionWrapper> move_base_client_;
};

}  // namespace Artemis
