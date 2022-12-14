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
 * @file artemis_node.cpp
 * @author Nitesh Jha (Navigator), Tanuj Thakkar (Driver)
 * @brief This file contains the main function for the artemis package
 *
 */

#include <Artemis/TaskAction.h>
#include <Artemis/WMSTask.h>
#include <ros/ros.h>

#include <task_action_client.hpp>
#include <wms_service_client.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "artemis_node");
  ros::NodeHandle nh;

  Artemis::WMSServiceClient wms_service_client(nh, "wms_server");
  Artemis::TaskActionClient task_action_client("task_server");

  while (ros::ok()) {
    Artemis::WMSTask wms_task;
    if (!wms_service_client.receiveTask(wms_task)) {
      ROS_INFO_STREAM("Artemis: Execution complete!");
      return 0;
    }

    Artemis::TaskGoal task_goal;
    task_goal.ID = wms_task.response.ID;
    task_goal.staging_goals = wms_task.response.staging_goals;
    task_goal.goal = wms_task.response.goal;

    task_action_client.requestTaskAction(task_goal);
    task_action_client.waitForResult();

    ros::Duration(1.0).sleep();
  }

  ros::spin();

  return 0;
}
