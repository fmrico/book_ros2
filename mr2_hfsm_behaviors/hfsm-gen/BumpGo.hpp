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


#ifndef MR2_HFSM_BEHAVIORS__HFSM_GEN__BUMPGO_HPP_
#define MR2_HFSM_BEHAVIORS__HFSM_GEN__BUMPGO_HPP_

#include <string>

#include "std_msgs/msg/string.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace cascade_hfsm
{
class BumpGo : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  BumpGo();
  virtual ~BumpGo();

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state);

  virtual void Turn_code_iterative() {}
  virtual void Turn_code_once() {}
  virtual void Forward_code_iterative() {}
  virtual void Forward_code_once() {}
  virtual void Back_code_iterative() {}
  virtual void Back_code_once() {}

  virtual bool Forward_2_Back() {return false;}
  virtual bool Back_2_Turn() {return false;}
  virtual bool Turn_2_Forward() {return false;}


  void tick();

protected:
  rclcpp::Time state_ts_;

private:
  void deactivateAllDeps();
  void Turn_activateDeps();
  void Forward_activateDeps();
  void Back_activateDeps();


  static const int TURN = 0;
  static const int FORWARD = 1;
  static const int BACK = 2;


  int state_;

  std::string myBaseId_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr loop_timer_;
};

}  // namespace cascade_hfsm

#endif  // MR2_HFSM_BEHAVIORS__HFSM_GEN__BUMPGO_HPP_
