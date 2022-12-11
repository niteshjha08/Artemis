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
#include <ros/ros.h>

namespace Artemis {

/**
 * @brief This class is used to define the WMS service server
 *
 */
class WMSServiceServer {
 private:
  ros::ServiceServer wms_service_server_;  // WMS service server

  /**
   * @brief This function is used to define the callback function for the WMS
   * service server
   *
   * @param req
   * @param res
   * @return true
   * @return false
   */
  bool assignTask(const Artemis::WMSTask::Request::ConstPtr req,
                  const Artemis::WMSTask::Response::ConstPtr res);

 public:
  /**
   * @brief Construct a new WMS Service Server object
   *
   */
  explicit WMSServiceServer(const ros::NodeHandle& node_handle);

  /**
   * @brief Destroy the WMS Service Server object
   *
   */
  ~WMSServiceServer();
};

}  // namespace Artemis
