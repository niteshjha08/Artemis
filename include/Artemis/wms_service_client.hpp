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
 * @file wms_service_client.hpp
 * @author Nitesh Jha (Navigator), Tanuj Thakkar (Driver)
 * @brief This file contains the class declaration for the WMS service client
 *
 */

#pragma once

#include <Artemis/WMSTask.h>
#include <ros/ros.h>

#include <string>

namespace Artemis {

/**
 * @brief This class is used to communicate with the WMS service server
 *
 */
class WMSServiceClient {
 public:
  /**
   * @brief Construct a new WMSServiceClient object
   *
   * @param node_handle The ROS node handle
   */
  WMSServiceClient(const ros::NodeHandle& node_handle,
                   const std::string& service_name);

  /**
   * @brief Destroy the WMSServiceClient object
   *
   */
  virtual ~WMSServiceClient();

  /**
   * @brief This function is used to send a task to the WMS service server
   *
   * @param task The task msg to be sent to the WMS service server
   * @return true If the task was successfully received from the WMS service
   * server
   * @return false If the task was not successfully received from the WMS
   * service server
   */
  bool receiveTask(Artemis::WMSTask& task);

 private:
  ros::NodeHandle node_handle_;            // ROS node handle
  const std::string service_name_;         // WMS service name
  ros::ServiceClient wms_service_client_;  // WMS service client
};

}  // namespace Artemis
