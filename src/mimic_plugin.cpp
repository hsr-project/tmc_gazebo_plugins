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
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/signals2.hpp>
#include <control_toolbox/pid.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>

namespace gazebo {

/// 各関節の最大出力トルク[Nm]
const double kDefaultMaxTorque = 200.0;

inline double clamp(double value, double min, double max) {
  assert(min < max);
  if (value < min) {
    return min;
  }
  if (value > max) {
    return max;
  }
  return value;
}

class MimicPlugin : public ModelPlugin {
 private:
  event::ConnectionPtr update_connection_;
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::JointPtr joint_;
  physics::JointPtr mimic_joint_;
  std::string joint_name_;
  std::string mimic_joint_name_;
  double multiplier_, offset_, sensitiveness_, max_torque_;
  // PID controller if needed
  control_toolbox::Pid pid_;
  boost::scoped_ptr<ros::NodeHandle> ros_node_;
  std::string robot_namespace_;

 public:
  MimicPlugin() {
    joint_.reset();
    mimic_joint_.reset();
  }

  ~MimicPlugin() {
#if GAZEBO_MAJOR_VERSION >= 8
    update_connection_.reset();
#else
    event::Events::DisconnectWorldUpdateBegin(update_connection_);
#endif
  }

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf ) {
    model_ = _parent;
    world_ = model_->GetWorld();
    robot_namespace_ = "";
    if (_sdf->HasElement("robot_namespace")) {
      robot_namespace_ = _sdf->GetElement("robot_namespace")->Get<std::string>();
    } else {
      // ネームスペースが指定されない場合、Gazebo中のモデル名からつける。
      std::string name = _parent->GetScopedName();
      std::vector<std::string> splitted;
      boost::split(splitted, name, boost::is_any_of("::"));
      robot_namespace_ = splitted.back();
    }
    ros_node_.reset(new ros::NodeHandle(robot_namespace_));
    joint_name_ = "joint";
    if (_sdf->HasElement("joint"))
      joint_name_ = _sdf->GetElement("joint")->Get<std::string>();

    mimic_joint_name_ = "mimicJoint";
    if (_sdf->HasElement("mimicJoint"))
      mimic_joint_name_ =
          _sdf->GetElement("mimicJoint")->Get<std::string>();

    if (_sdf->HasElement("multiplier")) {
      multiplier_ = _sdf->GetElement("multiplier")->Get<double>();
    } else {
      multiplier_ = 1.0;
    }

    if (_sdf->HasElement("offset")) {
      offset_ = _sdf->GetElement("offset")->Get<double>();
    } else {
      offset_ = 0.0;
    }

    if (_sdf->HasElement("sensitiveness")) {
      sensitiveness_ = _sdf->GetElement("sensitiveness")->Get<double>();
    } else {
      sensitiveness_ = 0.0;
    }

    if (_sdf->HasElement("max_torque")) {
      max_torque_ = _sdf->GetElement("max_torque")->Get<double>();
    } else {
      max_torque_ = kDefaultMaxTorque;
    }

    // Get the name of the parent model
    std::string model_name = _sdf->GetParent()->Get<std::string>("name");

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&gazebo::MimicPlugin::UpdateChild, this));
    gzdbg << "Plugin model name: " << model_name << std::endl;

    joint_ = model_->GetJoint(joint_name_);
    mimic_joint_ = model_->GetJoint(mimic_joint_name_);
    const ros::NodeHandle nh(*ros_node_, "mimic_controller/pid_gains/" + joint_name_);
    pid_.init(nh, true);
  }

  void UpdateChild() {
#if GAZEBO_MAJOR_VERSION >= 8
    static ros::Duration period(world_->Physics()->GetMaxStepSize());
    double angle = joint_->Position(0);
    double target_angle = mimic_joint_->Position(0) * multiplier_ + offset_;
#else
    static ros::Duration period(world_->GetPhysicsEngine()->GetMaxStepSize());
    double target_angle = mimic_joint_->GetAngle(0).Radian() * multiplier_ + offset_;
    double angle = joint_->GetAngle(0).Radian();
#endif
    double error = target_angle - angle;
    if (abs(error) >= sensitiveness_) {
      double effort = clamp(pid_.computeCommand(error, period), -max_torque_, max_torque_);
      joint_->SetForce(0, effort);
    }
    joint_->Update();
  }
};
GZ_REGISTER_MODEL_PLUGIN(MimicPlugin);
}  // namespace gazebo
