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

#include <algorithm>
#include <utility>

#include "br2_tracking/HeadController.hpp"
#include "br2_tracking/PIDController.hpp"

#include "br2_tracking_msgs/msg/pan_tilt_command.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace br2_tracking
{

using std::placeholders::_1;
using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

HeadController::HeadController()
: LifecycleNode("head_tracker"),
  pan_pid_(0.0, 1.0, 0.0, 0.3),
  tilt_pid_(0.0, 1.0, 0.0, 0.3)
{
  command_sub_ = create_subscription<br2_tracking_msgs::msg::PanTiltCommand>(
    "command", 100,
    std::bind(&HeadController::command_callback, this, _1));
  joint_sub_ = create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
    "joint_state", rclcpp::SensorDataQoS(),
    std::bind(&HeadController::joint_state_callback, this, _1));
  joint_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_command", 100);
}

CallbackReturn
HeadController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "HeadController configured");

  pan_pid_.set_pid(0.4, 0.05, 0.55);
  tilt_pid_.set_pid(0.4, 0.05, 0.55);

  return CallbackReturn::SUCCESS;
}

CallbackReturn
HeadController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "HeadController activated");

  joint_pub_->on_activate();
  timer_ = create_wall_timer(100ms, std::bind(&HeadController::control_sycle, this));

  return CallbackReturn::SUCCESS;
}

CallbackReturn
HeadController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "HeadController deactivated");

  trajectory_msgs::msg::JointTrajectory command_msg;
  command_msg.header.stamp = now();
  command_msg.joint_names = last_state_->joint_names;
  command_msg.points.resize(1);
  command_msg.points[0].positions.resize(2);
  command_msg.points[0].velocities.resize(2);
  command_msg.points[0].accelerations.resize(2);
  command_msg.points[0].positions[0] = 0.0;
  command_msg.points[0].positions[1] = 0.0;
  command_msg.points[0].velocities[0] = 0.0;
  command_msg.points[0].velocities[1] = 0.0;
  command_msg.points[0].accelerations[0] = 0.0;
  command_msg.points[0].accelerations[1] = 0.0;
  command_msg.points[0].time_from_start = rclcpp::Duration(1s);

  joint_pub_->publish(command_msg);

  joint_pub_->on_deactivate();
  timer_ = nullptr;

  return CallbackReturn::SUCCESS;
}

void
HeadController::joint_state_callback(
  control_msgs::msg::JointTrajectoryControllerState::UniquePtr msg)
{
  last_state_ = std::move(msg);
}

void
HeadController::command_callback(br2_tracking_msgs::msg::PanTiltCommand::UniquePtr msg)
{
  last_command_ = std::move(msg);
  last_command_ts_ = now();
}

void
HeadController::control_sycle()
{
  if (last_state_ == nullptr) {return;}

  trajectory_msgs::msg::JointTrajectory command_msg;
  command_msg.header.stamp = now();
  command_msg.joint_names = last_state_->joint_names;
  command_msg.points.resize(1);
  command_msg.points[0].positions.resize(2);
  command_msg.points[0].velocities.resize(2);
  command_msg.points[0].accelerations.resize(2);
  command_msg.points[0].time_from_start = rclcpp::Duration(200ms);

  if (last_command_ == nullptr || (now() - last_command_ts_) > 100ms) {
    command_msg.points[0].positions[0] = 0.0;
    command_msg.points[0].positions[1] = 0.0;
    command_msg.points[0].velocities[0] = 0.0;
    command_msg.points[0].velocities[1] = 0.0;
    command_msg.points[0].accelerations[0] = 0.0;
    command_msg.points[0].accelerations[1] = 0.0;
    command_msg.points[0].time_from_start = rclcpp::Duration(1s);
  } else {
    double control_pan = pan_pid_.get_output(last_command_->pan);
    double control_tilt = tilt_pid_.get_output(last_command_->tilt);

    command_msg.points[0].positions[0] = last_state_->feedback.positions[0] - control_pan;
    command_msg.points[0].positions[1] = last_state_->feedback.positions[1] - control_tilt;

    command_msg.points[0].velocities[0] = 0.0;
    command_msg.points[0].velocities[1] = 0.0;
    command_msg.points[0].accelerations[0] = 0.0;
    command_msg.points[0].accelerations[1] = 0.0;
  }

  joint_pub_->publish(command_msg);
}

}  // namespace br2_tracking
