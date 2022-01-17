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

#include <string>
#include <iostream>
#include <algorithm>

#include "br2_bt_patrolling/BatteryChecker.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace br2_bt_patrolling
{

using namespace std::chrono_literals;
using namespace std::placeholders;

BatteryChecker::BatteryChecker(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "/output_vel", 100, std::bind(&BatteryChecker::vel_callback, this, _1));

  last_reading_time_ = node_->now();
}

void
BatteryChecker::vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  float battery_level;
  if (!config().blackboard->get("battery_level", battery_level)) {
    battery_level = 100.0f;
  }

  float dt = (node_->now() - last_reading_time_).seconds();
  last_reading_time_ = node_->now();

  float vel = sqrt(msg->linear.x * msg->linear.x + msg->angular.z * msg->angular.z);

  battery_level = std::max(0.0f, battery_level - (vel * dt * DECAY_LEVEL) - EPSILON * dt);

  config().blackboard->set("battery_level", battery_level);
}

BT::NodeStatus
BatteryChecker::tick()
{
  float battery_level;
  if (!config().blackboard->get("battery_level", battery_level)) {
    battery_level = 100.0f;
  }

  std::cout << battery_level << std::endl;

  if (battery_level < MIN_LEVEL) {
    return BT::NodeStatus::FAILURE;
  } else {
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace br2_bt_patrolling

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<br2_bt_patrolling::BatteryChecker>("BatteryChecker");
}
