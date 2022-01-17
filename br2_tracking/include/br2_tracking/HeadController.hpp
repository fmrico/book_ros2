// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BR2_TRACKING__HEADCONTROLLER_HPP_
#define BR2_TRACKING__HEADCONTROLLER_HPP_

#include <memory>

#include "br2_tracking_msgs/msg/pan_tilt_command.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "br2_tracking/PIDController.hpp"

#include "image_transport/image_transport.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace br2_tracking
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HeadController : public rclcpp_lifecycle::LifecycleNode
{
public:
  HeadController();

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);

  void control_sycle();

  void joint_state_callback(control_msgs::msg::JointTrajectoryControllerState::UniquePtr msg);
  void command_callback(br2_tracking_msgs::msg::PanTiltCommand::UniquePtr msg);

private:
  rclcpp::Subscription<br2_tracking_msgs::msg::PanTiltCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joint_sub_;
  rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  control_msgs::msg::JointTrajectoryControllerState::UniquePtr last_state_;
  br2_tracking_msgs::msg::PanTiltCommand::UniquePtr last_command_;
  rclcpp::Time last_command_ts_;

  PIDController pan_pid_, tilt_pid_;
};

}  // namespace br2_tracking

#endif  // BR2_TRACKING__HEADCONTROLLER_HPP_
