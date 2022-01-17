// Copyright (c) 2018 Intel Corporation
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

#ifndef BR2_BT_PATROLLING__CTRL_SUPPORT__BTLIFECYCLECTRLNODE_HPP_
#define BR2_BT_PATROLLING__CTRL_SUPPORT__BTLIFECYCLECTRLNODE_HPP_

#include <memory>
#include <string>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace br2_bt_patrolling
{

using namespace std::chrono_literals;  // NOLINT

class BtLifecycleCtrlNode : public BT::ActionNodeBase
{
public:
  BtLifecycleCtrlNode(
    const std::string & xml_tag_name,
    const std::string & node_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(xml_tag_name, conf), ctrl_node_name_(node_name)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  }

  BtLifecycleCtrlNode() = delete;

  virtual ~BtLifecycleCtrlNode()
  {
  }

  template<typename serviceT>
  typename rclcpp::Client<serviceT>::SharedPtr createServiceClient(const std::string & service_name)
  {
    auto srv = node_->create_client<serviceT>(service_name);
    while (!srv->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      } else {
        RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
      }
    }
    return srv;
  }

  virtual void on_tick() {}

  virtual BT::NodeStatus on_success()
  {
    return BT::NodeStatus::SUCCESS;
  }

  virtual BT::NodeStatus on_failure()
  {
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus tick() override
  {
    if (status() == BT::NodeStatus::IDLE) {
      change_state_client_ = createServiceClient<lifecycle_msgs::srv::ChangeState>(
        ctrl_node_name_ + "/change_state");
      get_state_client_ = createServiceClient<lifecycle_msgs::srv::GetState>(
        ctrl_node_name_ + "/get_state");
    }

    if (ctrl_node_state_ != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      ctrl_node_state_ = get_state();
      set_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    }

    on_tick();

    return BT::NodeStatus::RUNNING;
  }

  void halt() override
  {
    if (ctrl_node_state_ == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      set_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    }
    setStatus(BT::NodeStatus::IDLE);
  }

  uint8_t get_state()
  {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto result = get_state_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, result) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      lifecycle_msgs::msg::State get_state;

      RCLCPP_ERROR(node_->get_logger(), "Failed to call get_state service");
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    return result.get()->current_state.id;
  }

  bool set_state(uint8_t state)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();

    if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
      ctrl_node_state_ == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    } else {
      if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
        ctrl_node_state_ == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
      } else {
        if (state != ctrl_node_state_) {
          RCLCPP_ERROR(
            node_->get_logger(), "Transition not possible %zu -> %zu", ctrl_node_state_, state);
          return false;
        } else {
          return true;
        }
      }
    }

    auto result = change_state_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, result) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to call set_state service");
      return false;
    }

    if (!result.get()->success) {
      RCLCPP_ERROR(
        node_->get_logger(), "Failed to set node state %zu -> %zu", ctrl_node_state_, state);
      return false;
    } else {
      RCLCPP_INFO(node_->get_logger(), "Transition success  %zu -> %zu", ctrl_node_state_, state);
    }

    ctrl_node_state_ = state;
    return true;
  }

  std::string ctrl_node_name_;
  uint8_t ctrl_node_state_;

  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client_;

  rclcpp::Node::SharedPtr node_;
};


}  // namespace br2_bt_patrolling

#endif  // BR2_BT_PATROLLING__CTRL_SUPPORT__BTLIFECYCLECTRLNODE_HPP_
