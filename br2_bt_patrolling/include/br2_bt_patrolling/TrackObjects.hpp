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

#ifndef BR2_BT_PATROLLING__TRACKOBJECTS_HPP_
#define BR2_BT_PATROLLING__TRACKOBJECTS_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "br2_bt_patrolling/ctrl_support/BTLifecycleCtrlNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace br2_bt_patrolling
{

class TrackObjects : public br2_bt_patrolling::BtLifecycleCtrlNode
{
public:
  explicit TrackObjects(
    const std::string & xml_tag_name,
    const std::string & node_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }
};

}  // namespace br2_bt_patrolling

#endif  // BR2_BT_PATROLLING__TRACKOBJECTS_HPP_
