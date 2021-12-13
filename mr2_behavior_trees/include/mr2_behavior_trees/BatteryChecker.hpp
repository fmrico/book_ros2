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

#ifndef MR2_BEHAVIOR_TREES__BATTERYCHECKER_HPP_
#define MR2_BEHAVIOR_TREES__BATTERYCHECKER_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace mr2_behavior_trees
{

class BatteryChecker : public BT::ActionNodeBase
{
public:
  explicit BatteryChecker(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

  void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  const float DECAY_LEVEL = 0.5;  // 0.5 * |vel| * dt
  const float EPSILON = 0.01;  // 0.001 * dt
  const float MIN_LEVEL = 10.0;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time last_reading_time_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
};

}  // namespace mr2_behavior_trees

#endif  // MR2_BEHAVIOR_TREES__BATTERYCHECKER_HPP_
