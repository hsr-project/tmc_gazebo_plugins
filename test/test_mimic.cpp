/*
Copyright (c) 2016 TOYOTA MOTOR CORPORATION
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#include <gtest/gtest.h>
#include <tinyxml.h>
#include "gazebo_msgs/SpawnModel.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

#define TOL_RAD 0.01

double g_pos_jointone = 0.000001;
double g_pos_jointtwo = 0.000001;
bool g_all_joints_data_received = false;

void GetJointStates(const sensor_msgs::JointState::ConstPtr &msg) {
  for (unsigned int i = 0 ; i < msg->name.size() ; ++i) {
    if (msg->name[i] == "jointone") {
      g_pos_jointone = msg->position[i];
    } else if (msg->name[i] == "jointtwo") {
      g_pos_jointtwo = msg->position[i];
    }
  }
  g_all_joints_data_received = true;
}

TEST(Imu, publishedValues) {
  ros::NodeHandle n;

  // publish joint position
  ros::Publisher pub = n.advertise<std_msgs::Float64>(
    "boxes_mimic/jointone_controller/command", 1000);

  // subscribe to joint state
  ros::Subscriber sub =
    n.subscribe("boxes_mimic/joint_states", 1000, &GetJointStates);

  // wait for state
  ros::Rate loop_rate(100.0);
  while (ros::ok()) {
    if (g_all_joints_data_received)
      break;
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("Please wait around 10s");

  // control joint one and check joint two
  double ref = 0.0;
  while (ros::ok()) {
    std_msgs::Float64 msg;
    ref += 0.0001;
    msg.data = ref;
    pub.publish(msg);

    if (ref >= 0.1)
      break;

    ros::spinOnce();
    loop_rate.sleep();
  }

  // check
  EXPECT_NEAR(g_pos_jointtwo, g_pos_jointone*0.5, TOL_RAD);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_mimic");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
