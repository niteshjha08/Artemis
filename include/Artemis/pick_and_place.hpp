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
 * @author Nitesh Jha (Driver), Tanuj Thakkar (Navigator)
 * @brief This file contains the declaration of the PickAndPlace class
 *
 */

#pragma once

#include <ros/ros.h>
#include <string>
#include <std_msgs/Float64.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <gazebo_ros_link_attacher/AttachRequest.h>
#include <gazebo_ros_link_attacher/AttachResponse.h>


namespace Artemis {

    class PickAndPlace {
    public:
    PickAndPlace(const ros::NodeHandle& node_handle);

    ~PickAndPlace();

    void pickCargo(int task_id);

    void placeCargo(int task_id);

    private:
        ros::NodeHandle node_handle_;       // NodeHandle for the node
        ros::Publisher shoulder_pub_;     // Publisher for shoulder joint
        ros::Publisher elbow_pub_;        // Publisher for elbow joint
        ros::ServiceClient attach_client_;  // Service client for attaching
        ros::ServiceClient detach_client_;  // Service client for detaching
};

}
