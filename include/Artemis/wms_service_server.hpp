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
 * @file wms_service_server.hpp
 * @author Nitesh Jha (Navigator), Tanuj Thakkar (Driver)
 * @brief This file contains the class declaration for the WMS service server
 *
 */

#pragma once

#include <Artemis/WMSTask.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>

#include <random>
#include <unordered_map>
#include <vector>
#include <string>

namespace Artemis {

/**
 * @brief This class is used to define the WMS service server
 *
 */
class WMSServiceServer {
 public:
  /**
   * @brief Construct a new WMS Service Server object
   *
   */
  WMSServiceServer(const ros::NodeHandle& node_handle,
                   const std::string& service_name);

  /**
   * @brief Destroy the WMS Service Server object
   *
   */
  ~WMSServiceServer();

  /**
   * @brief This function is used to define the callback function for the WMS
   * service server
   *
   * @param req
   * @param res
   * @return true
   * @return false
   */
  bool assignTask(Artemis::WMSTask::Request& req,
                  Artemis::WMSTask::Response& res);

 private:
  ros::NodeHandle node_handle_;                 // Node handle
  const std::string service_name_;              // Service name
  ros::ServiceServer wms_service_server_;       // WMS service server
  std::vector<int> task_queue_ = {1, 2, 3, 4};  // Task queue
  const std::vector<std::vector<float>> staging_goals_ = {
      {1.750, 5.650, 0., 0., 0., -1.00, 0.},
      {-1.000, 1.950, 0., 0., 0., 0.700, 0.713},
      {-4.335, 5.365, 0., 0., 0., 0., 1.000},
      {-1.061, 9.029, 0., 0., 0., -0.700, 0.700}};  // Staging goals
  const std::unordered_map<int, std::vector<float>> goals_ = {
      {1, {5.000, 0.500, 0., 0., 0., 0., 1.0}},
      {2, {5.000, 0., 0., 0., 0., 0., 1.0}},
      {3, {5.000, -0.500, 0., 0., 0., 0., 1.0}},
      {4, {5.000, -1.000, 0., 0., 0., 0., 1.0}}};  // Goals
};

}  // namespace Artemis
