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
 * @file wms_service_client.cpp
 * @author Nitesh Jha (Navigator), Tanuj Thakkar (Driver)
 * @brief This file contains the class defintion for the WMS service client
 *
 */

#include <wms_service_client.hpp>

namespace Artemis {

/**
 * @brief Construct a new WMSServiceClient::WMSServiceClient object
 *
 * @param node_handle Node handle for the ROS node
 */
WMSServiceClient::WMSServiceClient(const ros::NodeHandle& node_handle,
                                   const std::string& service_name)
    : node_handle_(node_handle),
      service_name_(service_name),
      wms_service_client_(
          node_handle_.serviceClient<Artemis::WMSTask>(service_name_)) {
  while (!ros::service::waitForService(service_name, ros::Duration(1.0))) {
    ROS_INFO_STREAM("WMSServiceClient ("
                    << service_name_
                    << "): Waiting for the WMS service server to start");
  }
  ROS_INFO_STREAM("WMSServiceClient (" << service_name_
                                       << "): WMS service client created");
}

/**
 * @brief Destroy the WMSServiceClient::WMSServiceClient object
 *
 */
WMSServiceClient::~WMSServiceClient() {
  ROS_INFO_STREAM("WMSServiceClient (" << service_name_
                                       << "): WMS service client destroyed");
}

/**
 * @brief This function is used to send a task to the WMS service server
 *
 * @return true Received task successfully
 * @return false Failed to receive task
 */
bool WMSServiceClient::receiveTask(Artemis::WMSTask& task) {
  if (wms_service_client_.call(task)) {
    ROS_INFO_STREAM("WMSServiceClient (" << service_name_
                                         << "): Received task successfully");
    return true;
  } else {
    ROS_INFO_STREAM("WMSServiceClient (" << service_name_
                                         << "): Failed to receive task");
    return false;
  }
}

}  // namespace Artemis
