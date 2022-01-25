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
#include <set>

#include "br2_bt_patrolling/Recharge.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace br2_bt_patrolling
{

Recharge::Recharge(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
{
}

void
Recharge::halt()
{
}

BT::NodeStatus
Recharge::tick()
{
  std::cout << "Recharge tick " << counter_ << std::endl;

  if (counter_++ < 50) {
    return BT::NodeStatus::RUNNING;
  } else {
    counter_ = 0;
    config().blackboard->set<float>("battery_level", 100.0f);
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace br2_bt_patrolling

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<br2_bt_patrolling::Recharge>("Recharge");
}
