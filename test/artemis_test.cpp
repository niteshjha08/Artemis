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
 * @file test.cpp
 * @author Nitesh Jha (Navigator), Tanuj Thakkar (Driver)
 * @brief This file contains the tests for the Artemis package
 *
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <wms_service_server.hpp>
#include <pick_and_place.hpp>
#include <task_action_server.hpp>


TEST(WMSServer, test_wms_server) {
  ros::NodeHandle nh;
  const std::string service_name = "wms_server";
  Artemis::WMSServiceServer wms_service_server(nh, service_name);
  ASSERT_NO_THROW(wms_service_server);
}

TEST(PickPlace, pick_test) {
  ros::NodeHandle nh;
  Artemis::PickAndPlace pick_and_place_(nh);
  ASSERT_NO_THROW(pick_and_place_);
}

TEST(PickPlace, place_test) {
  ros::NodeHandle nh;
  Artemis::PickAndPlace pick_and_place_(nh);
  ASSERT_NO_THROW(pick_and_place_.placeCargo(2));
}


TEST(task_Action_server, action_server) {
  ros::NodeHandle nh;
  const std::string action_name = "action_server";
  Artemis::TaskActionServer asrv(nh, action_name);
  const Artemis::TaskGoalConstPtr task_goal;
  ASSERT_NO_THROW(asrv.executeTask(task_goal));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "artemis_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
