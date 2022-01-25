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
#include <list>
#include <memory>
#include <vector>
#include <set>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "gtest/gtest.h"

using namespace std::placeholders;
using namespace std::chrono_literals;


class VelocitySinkNode : public rclcpp::Node
{
public:
  VelocitySinkNode()
  : Node("VelocitySink")
  {
    vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/output_vel", 100, std::bind(&VelocitySinkNode::vel_callback, this, _1));
  }

  void vel_callback(geometry_msgs::msg::Twist::SharedPtr msg)
  {
    vel_msgs_.push_back(*msg);
  }

  std::list<geometry_msgs::msg::Twist> vel_msgs_;

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
};


TEST(bt_action, turn_btn)
{
  auto node = rclcpp::Node::make_shared("turn_btn_node");
  auto node_sink = std::make_shared<VelocitySinkNode>();

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("br2_turn_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <Turn />
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  rclcpp::Rate rate(10);
  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
    rclcpp::spin_some(node_sink);
    rate.sleep();
  }

  ASSERT_FALSE(node_sink->vel_msgs_.empty());
  ASSERT_NEAR(node_sink->vel_msgs_.size(), 30, 1);

  geometry_msgs::msg::Twist & one_twist = node_sink->vel_msgs_.front();

  ASSERT_GT(one_twist.angular.z, 0.1);
  ASSERT_NEAR(one_twist.linear.x, 0.0, 0.0000001);
}

TEST(bt_action, back_btn)
{
  auto node = rclcpp::Node::make_shared("back_btn_node");
  auto node_sink = std::make_shared<VelocitySinkNode>();

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("br2_back_bt_node"));

  std::string xml_bt =gte_node
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <Back />
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  rclcpp::Rate rate(10);
  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
    rclcpp::spin_some(node_sink);
    rate.sleep();
  }

  ASSERT_FALSE(node_sink->vel_msgs_.empty());
  ASSERT_NEAR(node_sink->vel_msgs_.size(), 30, 1);

  geometry_msgs::msg::Twist & one_twist = node_sink->vel_msgs_.front();

  ASSERT_LT(one_twist.linear.x, -0.1);
  ASSERT_NEAR(one_twist.angular.z, 0.0, 0.0000001);
}

TEST(bt_action, forward_btn)
{
  auto node = rclcpp::Node::make_shared("forward_btn_node");
  auto node_sink = std::make_shared<VelocitySinkNode>();

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("br2_forward_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <Forward />
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  rclcpp::Rate rate(10);
  auto current_status = BT::NodeStatus::FAILURE;
  int counter = 0;
  while (counter++ < 30 && rclcpp::ok()) {
    current_status = tree.rootNode()->executeTick();
    rclcpp::spin_some(node_sink);
    rate.sleep();
  }

  ASSERT_EQ(current_status, BT::NodeStatus::RUNNING);
  ASSERT_FALSE(node_sink->vel_msgs_.empty());
  ASSERT_NEAR(node_sink->vel_msgs_.size(), 30, 1);

  geometry_msgs::msg::Twist & one_twist = node_sink->vel_msgs_.front();

  ASSERT_GT(one_twist.linear.x, 0.1);
  ASSERT_NEAR(one_twist.angular.z, 0.0, 0.0000001);
}

TEST(bt_action, is_obstacle_btn)
{
  auto node = rclcpp::Node::make_shared("is_obstacle_btn_node");
  auto scan_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("input_scan", 1);

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("br2_is_obstacle_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <IsObstacle/>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  rclcpp::Rate rate(10);

  sensor_msgs::msg::LaserScan scan;
  scan.ranges.push_back(2.0);
  for (int i = 0; i < 10; i++) {
    scan_pub->publish(scan);
    rclcpp::spin_some(node);
    rate.sleep();
  }

  BT::NodeStatus current_status = tree.rootNode()->executeTick();
  ASSERT_EQ(current_status, BT::NodeStatus::FAILURE);

  scan.ranges[0] = 0.3;
  for (int i = 0; i < 10; i++) {
    scan_pub->publish(scan);
    rclcpp::spin_some(node);
    rate.sleep();
  }

  current_status = tree.rootNode()->executeTick();
  ASSERT_EQ(current_status, BT::NodeStatus::SUCCESS);

  xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <IsObstacle distance="0.5"/>
      </BehaviorTree>
    </root>)";
  tree = factory.createTreeFromText(xml_bt, blackboard);

  scan.ranges[0] = 0.3;
  for (int i = 0; i < 10; i++) {
    scan_pub->publish(scan);
    rclcpp::spin_some(node);
    rate.sleep();
  }

  current_status = tree.rootNode()->executeTick();
  ASSERT_EQ(current_status, BT::NodeStatus::SUCCESS);

  scan.ranges[0] = 0.6;
  for (int i = 0; i < 10; i++) {
    scan_pub->publish(scan);
    rclcpp::spin_some(node);
    rate.sleep();
  }

  current_status = tree.rootNode()->executeTick();
  ASSERT_EQ(current_status, BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
