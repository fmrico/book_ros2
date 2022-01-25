// Copyright 2019 Intelligent Robotics Lab
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
#include <memory>

#include "br2_bt_patrolling/TrackObjects.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace br2_bt_patrolling
{

TrackObjects::TrackObjects(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: br2_bt_patrolling::BtLifecycleCtrlNode(xml_tag_name, action_name, conf)
{
}

}  // namespace br2_bt_patrolling

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<br2_bt_patrolling::TrackObjects>(
        name, "/head_tracker", config);
    };

  factory.registerBuilder<br2_bt_patrolling::TrackObjects>(
    "TrackObjects", builder);
}
