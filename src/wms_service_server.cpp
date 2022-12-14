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

#include <wms_service_server.hpp>

namespace Artemis {

WMSServiceServer::WMSServiceServer(const ros::NodeHandle& node_handle,
                                   const std::string& service_name)
    : node_handle_(node_handle),
      service_name_(service_name),
      wms_service_server_(node_handle_.advertiseService(
          service_name_, &WMSServiceServer::assignTask, this)) {
  ROS_INFO_STREAM("WMSServiceServer (" << service_name_
                                       << "): WMS Service Server is running");
  std::shuffle(std::begin(task_queue_), std::end(task_queue_),
               std::default_random_engine(std::random_device()()));

  ROS_INFO_STREAM("WMSServiceServer ("
                  << service_name_ << "): Task Queue: " << task_queue_[0] << " "
                  << task_queue_[1] << " " << task_queue_[2] << " "
                  << task_queue_[3]);
}

WMSServiceServer::~WMSServiceServer() {
  ROS_INFO_STREAM("WMSServiceServer ("
                  << service_name_ << "): WMS Service Server is shutting down");
}

bool WMSServiceServer::assignTask(Artemis::WMSTask::Request& req,
                                  Artemis::WMSTask::Response& res) {
  if (task_queue_.empty()) {
    ROS_INFO_STREAM("WMSServiceServer (" << service_name_
                                         << "): No tasks available");
    return false;
  } else {
    res.ID = task_queue_.front();

    res.staging_goals.header.frame_id = "map";
    res.staging_goals.header.stamp = ros::Time::now();
    for (const auto& staging_goal : staging_goals_) {
      geometry_msgs::Pose pose;
      pose.position.x = staging_goal[0];
      pose.position.y = staging_goal[1];
      pose.position.z = staging_goal[2];
      pose.orientation.x = staging_goal[3];
      pose.orientation.y = staging_goal[4];
      pose.orientation.z = staging_goal[5];
      pose.orientation.w = staging_goal[6];
      res.staging_goals.poses.push_back(pose);
    }

    res.goal.position.x = goals_.at(task_queue_.front())[0];
    res.goal.position.y = goals_.at(task_queue_.front())[1];
    res.goal.position.z = goals_.at(task_queue_.front())[2];
    res.goal.orientation.x = goals_.at(task_queue_.front())[3];
    res.goal.orientation.y = goals_.at(task_queue_.front())[4];
    res.goal.orientation.z = goals_.at(task_queue_.front())[5];
    res.goal.orientation.w = goals_.at(task_queue_.front())[6];

    task_queue_.erase(task_queue_.begin());
    ROS_INFO_STREAM("WMSServiceServer ("
                    << service_name_
                    << "): Task assigned: " << static_cast<int>(res.ID));
    return true;
  }

  return false;
}

}  // namespace Artemis
