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
#include <vector>

#include "mr2_behavior_trees/GetWaypoint.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

namespace mr2_behavior_trees
{

int GetWaypoint::current_ = 0;

GetWaypoint::GetWaypoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  std::vector<std::string> waypoints;
  node->get_parameter("waypoints", waypoints);

  geometry_msgs::msg::PoseStamped wp;
  wp.header.frame_id = "map";
  wp.pose.orientation.w = 1.0;

  for (const auto & waypoint_id : waypoints) {
    std::vector<double> wp_coords;
    node->get_parameter(waypoint_id, wp_coords);
    wp.pose.position.x = wp_coords[0];
    wp.pose.position.y = wp_coords[1];

    if (waypoint_id == "recharge") {
      recharge_point_ = wp;
    } else {
      waypoints_.push_back(wp);
    }
  }
}

void
GetWaypoint::halt()
{
  std::cout << "GetWaypoint halt" << std::endl;
}

BT::NodeStatus
GetWaypoint::tick()
{
  std::string id;
  getInput("wp_id", id);

  if (id == "recharge") {
    setOutput("waypoint", recharge_point_);
  } else {
    setOutput("waypoint", waypoints_[current_++]);
    current_ = current_ % waypoints_.size();
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace mr2_behavior_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mr2_behavior_trees::GetWaypoint>("GetWaypoint");
}
