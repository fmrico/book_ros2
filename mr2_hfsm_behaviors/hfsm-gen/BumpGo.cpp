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


#include "BumpGo.hpp"

namespace cascade_hfsm
{
BumpGo::BumpGo()
: CascadeLifecycleNode("BumpGo"), state_(FORWARD), myBaseId_("BumpGo")
{
  declare_parameter("frequency");

  state_ts_ = now();
  state_pub_ = create_publisher<std_msgs::msg::String>("/" + myBaseId_ + "/state", 1);
}

BumpGo::~BumpGo()
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BumpGo::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  deactivateAllDeps();

  state_ = FORWARD;
  state_ts_ = now();

  Forward_activateDeps();
  Forward_code_once();


  double frequency = 5.0;
  get_parameter_or<double>("frequency", frequency, 5.0);

  loop_timer_ = create_wall_timer(
    std::chrono::duration<double, std::ratio<1>>(1.0 / frequency),
    std::bind(&BumpGo::tick, this));

  state_pub_->on_activate();
  return CascadeLifecycleNode::on_activate(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BumpGo::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  loop_timer_ = nullptr;

  return CascadeLifecycleNode::on_deactivate(previous_state);
}

void BumpGo::tick()
{
  std_msgs::msg::String msg;

  switch (state_) {
    case TURN:
      Turn_code_iterative();

      msg.data = "Turn";
      state_pub_->publish(msg);

      if (Turn_2_Forward()) {
        deactivateAllDeps();

        state_ = FORWARD;
        state_ts_ = now();

        Forward_activateDeps();
        Forward_code_once();
      }
      break;
    case FORWARD:
      Forward_code_iterative();

      msg.data = "Forward";
      state_pub_->publish(msg);

      if (Forward_2_Back()) {
        deactivateAllDeps();

        state_ = BACK;
        state_ts_ = now();

        Back_activateDeps();
        Back_code_once();
      }
      break;
    case BACK:
      Back_code_iterative();

      msg.data = "Back";
      state_pub_->publish(msg);

      if (Back_2_Turn()) {
        deactivateAllDeps();

        state_ = TURN;
        state_ts_ = now();

        Turn_activateDeps();
        Turn_code_once();
      }
      break;
  }
}

void
BumpGo::deactivateAllDeps()
{
}

void
BumpGo::Turn_activateDeps()
{
}
void
BumpGo::Forward_activateDeps()
{
}
void
BumpGo::Back_activateDeps()
{
}


}  // namespace cascade_hfsm
