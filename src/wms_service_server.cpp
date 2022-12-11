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
 * @file wms_service_server.cpp
 * @author Nitesh Jha (Navigator), Tanuj Thakkar (Driver)
 * @brief This file contains the class definition for the WMS service server
 *
 */

#pragma once

#include <ros/ros.h>

#include <wms_service_server.hpp>

namespace Artemis {

/**
 * @brief This function is used to define the callback function for the WMS
 * service server
 *
 * @param req
 * @param res
 * @return true
 * @return false
 */
bool WMSServiceServer::assignTask(
    const Artemis::WMSTask::Request::ConstPtr req,
    const Artemis::WMSTask::Response::ConstPtr res) {
  return true;
}

/**
 * @brief This function is used to define the constructor for the WMS service
 * server
 *
 * @param node_handle
 */
WMSServiceServer::WMSServiceServer(const ros::NodeHandle& node_handle) {}

/**
 * @brief This function is used to define the destructor for the WMS service
 * server
 *
 */
WMSServiceServer::~WMSServiceServer() {}

}  // namespace Artemis
