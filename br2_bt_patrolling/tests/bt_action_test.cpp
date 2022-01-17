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
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "br2_bt_patrolling/TrackObjects.hpp"

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

class Nav2FakeServer : public rclcpp::Node
{
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

public:
  Nav2FakeServer()
  : Node("nav2_fake_server_node") {}

  void start_server()
  {
    move_action_server_ = rclcpp_action::create_server<NavigateToPose>(
      shared_from_this(),
      "navigate_to_pose",
      std::bind(&Nav2FakeServer::handle_goal, this, _1, _2),
      std::bind(&Nav2FakeServer::handle_cancel, this, _1),
      std::bind(&Nav2FakeServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<NavigateToPose>::SharedPtr move_action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    std::thread{std::bind(&Nav2FakeServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    auto result = std::make_shared<NavigateToPose::Result>();

    auto start = now();

    while ((now() - start) < 5s) {
      feedback->distance_remaining = 5.0 - (now() - start).seconds();
      goal_handle->publish_feedback(feedback);
    }

    goal_handle->succeed(result);
  }
};

class StoreWP : public BT::ActionNodeBase
{
public:
  explicit StoreWP(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(xml_tag_name, conf) {}

  void halt() {}
  BT::NodeStatus tick()
  {
    waypoints_.push_back(getInput<geometry_msgs::msg::PoseStamped>("in").value());
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
    {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("in")
    });
  }

  static std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
};

std::vector<geometry_msgs::msg::PoseStamped> StoreWP::waypoints_;

TEST(bt_action, recharge_btn)
{
  auto node = rclcpp::Node::make_shared("recharge_btn_node");

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("br2_recharge_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <Recharge    name="recharge"/>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  rclcpp::Rate rate(10);

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
    rate.sleep();
  }

  float battery_level;
  ASSERT_TRUE(blackboard->get("battery_level", battery_level));
  ASSERT_NEAR(battery_level, 100.0f, 0.0000001);
}

TEST(bt_action, patrol_btn)
{
  auto node = rclcpp::Node::make_shared("patrol_btn_node");
  auto node_sink = std::make_shared<VelocitySinkNode>();

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("br2_patrol_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <Patrol    name="patrol"/>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  rclcpp::Rate rate(10);

  bool finish = false;
  int counter = 0;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
    rclcpp::spin_some(node_sink->get_node_base_interface());
    rate.sleep();
  }

  ASSERT_FALSE(node_sink->vel_msgs_.empty());
  ASSERT_NEAR(node_sink->vel_msgs_.size(), 150, 2);

  geometry_msgs::msg::Twist & one_twist = node_sink->vel_msgs_.front();

  ASSERT_GT(one_twist.angular.z, 0.1);
  ASSERT_NEAR(one_twist.linear.x, 0.0, 0.0000001);
}

TEST(bt_action, move_btn)
{
  auto node = rclcpp::Node::make_shared("move_btn_node");
  auto nav2_fake_node = std::make_shared<Nav2FakeServer>();

  nav2_fake_node->start_server();

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {rclcpp::spin_some(nav2_fake_node);}
    });


  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("br2_move_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <Move    name="move" goal="{goal}"/>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);

  geometry_msgs::msg::PoseStamped goal;
  blackboard->set("goal", goal);

  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  rclcpp::Rate rate(10);

  int counter = 0;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
    rate.sleep();
  }

  t.join();
}

TEST(bt_action, get_waypoint_btn)
{
  auto node = rclcpp::Node::make_shared("get_waypoint_btn_node");

  rclcpp::spin_some(node);

  {
    BT::BehaviorTreeFactory factory;
    BT::SharedLibrary loader;

    factory.registerFromPlugin(loader.getOSName("br2_get_waypoint_bt_node"));

    std::string xml_bt =
      R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <GetWaypoint    name="recharge" wp_id="{id}" waypoint="{waypoint}"/>
        </BehaviorTree>
      </root>)";

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    blackboard->set<std::string>("id", "recharge");

    BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

    rclcpp::Rate rate(10);

    bool finish = false;
    int counter = 0;
    while (!finish && rclcpp::ok()) {
      finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
      counter++;
      rate.sleep();
    }

    auto point = blackboard->get<geometry_msgs::msg::PoseStamped>("waypoint");

    ASSERT_EQ(counter, 1);
    ASSERT_NEAR(point.pose.position.x, 3.67, 0.0000001);
    ASSERT_NEAR(point.pose.position.y, -0.24, 0.0000001);
  }

  {
    BT::BehaviorTreeFactory factory;
    BT::SharedLibrary loader;

    factory.registerNodeType<StoreWP>("StoreWP");
    factory.registerFromPlugin(loader.getOSName("br2_get_waypoint_bt_node"));

    std::string xml_bt =
      R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <Sequence name="root_sequence">
             <GetWaypoint    name="wp1" wp_id="next" waypoint="{waypoint}"/>
             <StoreWP in="{waypoint}"/>
             <GetWaypoint    name="wp2" wp_id="next" waypoint="{waypoint}"/>
             <StoreWP in="{waypoint}"/>
             <GetWaypoint    name="wp3" wp_id="" waypoint="{waypoint}"/>
             <StoreWP in="{waypoint}"/>
             <GetWaypoint    name="wp4" wp_id="recharge" waypoint="{waypoint}"/>
             <StoreWP in="{waypoint}"/>
             <GetWaypoint    name="wp5" wp_id="wp1" waypoint="{waypoint}"/>
             <StoreWP in="{waypoint}"/>
             <GetWaypoint    name="wp6" wp_id="wp2" waypoint="{waypoint}"/>
             <StoreWP in="{waypoint}"/>
             <GetWaypoint    name="wpt" waypoint="{waypoint}"/>
             <StoreWP in="{waypoint}"/>
          </Sequence>
        </BehaviorTree>
      </root>)";

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);

    BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

    rclcpp::Rate rate(10);

    bool finish = false;
    while (!finish && rclcpp::ok()) {
      finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
      rate.sleep();
    }

    const auto & waypoints = StoreWP::waypoints_;
    ASSERT_EQ(waypoints.size(), 7);
    ASSERT_NEAR(waypoints[0].pose.position.x, 1.07, 0.0000001);
    ASSERT_NEAR(waypoints[0].pose.position.y, -12.38, 0.0000001);
    ASSERT_NEAR(waypoints[1].pose.position.x, -5.32, 0.0000001);
    ASSERT_NEAR(waypoints[1].pose.position.y, -8.85, 0.0000001);
    ASSERT_NEAR(waypoints[2].pose.position.x, -0.56, 0.0000001);
    ASSERT_NEAR(waypoints[2].pose.position.y, 0.24, 0.0000001);

    ASSERT_NEAR(waypoints[3].pose.position.x, 3.67, 0.0000001);
    ASSERT_NEAR(waypoints[3].pose.position.y, -0.24, 0.0000001);

    ASSERT_NEAR(waypoints[4].pose.position.x, 1.07, 0.0000001);
    ASSERT_NEAR(waypoints[4].pose.position.y, -12.38, 0.0000001);
    ASSERT_NEAR(waypoints[5].pose.position.x, -5.32, 0.0000001);
    ASSERT_NEAR(waypoints[5].pose.position.y, -8.85, 0.0000001);
    ASSERT_NEAR(waypoints[6].pose.position.x, -0.56, 0.0000001);
    ASSERT_NEAR(waypoints[6].pose.position.y, 0.24, 0.0000001);
  }
}

TEST(bt_action, battery_checker_btn)
{
  auto node = rclcpp::Node::make_shared("battery_checker_btn_node");
  auto vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/output_vel", 100);

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("br2_battery_checker_bt_node"));
  factory.registerFromPlugin(loader.getOSName("br2_patrol_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <ReactiveSequence>
              <BatteryChecker    name="battery_checker"/>
              <Patrol    name="patrol"/>
          </ReactiveSequence>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  rclcpp::Rate rate(10);
  geometry_msgs::msg::Twist vel;
  vel.linear.x = 0.8;

  bool finish = false;
  int counter = 0;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    vel_pub->publish(vel);

    rclcpp::spin_some(node);
    rate.sleep();
  }

  float battery_level;
  ASSERT_TRUE(blackboard->get("battery_level", battery_level));
  ASSERT_NEAR(battery_level, 94.6, 1.0);
}

TEST(bt_action, track_objects_btn_1)
{
  auto node = rclcpp::Node::make_shared("track_objects_btn_node");
  auto node_head_tracker = rclcpp_lifecycle::LifecycleNode::make_shared("head_tracker");

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {rclcpp::spin_some(node_head_tracker->get_node_base_interface());}
    });

  BT::NodeConfiguration conf;
  conf.blackboard = BT::Blackboard::create();
  conf.blackboard->set("node", node);
  br2_bt_patrolling::BtLifecycleCtrlNode bt_node("TrackObjects", "head_tracker", conf);

  bt_node.change_state_client_ = bt_node.createServiceClient<lifecycle_msgs::srv::ChangeState>(
    "/head_tracker/change_state");
  ASSERT_TRUE(bt_node.change_state_client_->service_is_ready());

  bt_node.get_state_client_ = bt_node.createServiceClient<lifecycle_msgs::srv::GetState>(
    "/head_tracker/get_state");
  ASSERT_TRUE(bt_node.get_state_client_->service_is_ready());
  auto start = node->now();

  rclcpp::Rate rate(10);
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  ASSERT_EQ(bt_node.get_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  bt_node.ctrl_node_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
  ASSERT_FALSE(bt_node.set_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE));

  node_head_tracker->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  bt_node.ctrl_node_state_ = bt_node.get_state();

  ASSERT_TRUE(bt_node.set_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE));
  ASSERT_EQ(bt_node.get_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  bt_node.ctrl_node_state_ = bt_node.get_state();

  ASSERT_TRUE(bt_node.set_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE));
  ASSERT_EQ(bt_node.get_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  finish = true;
  t.join();
}

TEST(bt_action, track_objects_btn_2)
{
  auto node = rclcpp::Node::make_shared("track_objects_btn_node");
  auto node_head_tracker = rclcpp_lifecycle::LifecycleNode::make_shared("head_tracker");

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {rclcpp::spin_some(node_head_tracker->get_node_base_interface());}
    });

  BT::NodeConfiguration conf;
  conf.blackboard = BT::Blackboard::create();
  conf.blackboard->set("node", node);
  br2_bt_patrolling::BtLifecycleCtrlNode bt_node("TrackObjects", "head_tracker", conf);

  node_head_tracker->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::Rate rate(10);
  auto start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  ASSERT_EQ(bt_node.tick(), BT::NodeStatus::RUNNING);

  ASSERT_TRUE(bt_node.change_state_client_->service_is_ready());
  ASSERT_TRUE(bt_node.get_state_client_->service_is_ready());

  ASSERT_EQ(bt_node.get_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_EQ(bt_node.tick(), BT::NodeStatus::RUNNING);

  bt_node.halt();

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  ASSERT_EQ(bt_node.get_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  finish = true;
  t.join();
}

TEST(bt_action, track_objects_btn_3)
{
  auto node = rclcpp::Node::make_shared("track_objects_btn_node");
  auto node_head_tracker = rclcpp_lifecycle::LifecycleNode::make_shared("head_tracker");

  node_head_tracker->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {rclcpp::spin_some(node_head_tracker->get_node_base_interface());}
    });

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("br2_track_objects_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <KeepRunningUntilFailure>
              <TrackObjects    name="track_objects"/>
          </KeepRunningUntilFailure>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  auto start = node->now();
  rclcpp::Rate rate(10);

  {
    BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

    ASSERT_EQ(
      node_head_tracker->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    while (rclcpp::ok() && (node->now() - start) < 1s) {
      tree.rootNode()->executeTick() == BT::NodeStatus::RUNNING;

      rclcpp::spin_some(node);
      rate.sleep();
    }
    ASSERT_EQ(
      node_head_tracker->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  ASSERT_EQ(
    node_head_tracker->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  finish = true;
  t.join();
}

TEST(bt_action, move_track_btn)
{
  auto node = rclcpp::Node::make_shared("move_btn_node");
  auto nav2_fake_node = std::make_shared<Nav2FakeServer>();
  auto node_head_tracker = rclcpp_lifecycle::LifecycleNode::make_shared("head_tracker");

  node_head_tracker->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  nav2_fake_node->start_server();

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(nav2_fake_node);
  exe.add_node(node_head_tracker->get_node_base_interface());
  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("br2_move_bt_node"));
  factory.registerFromPlugin(loader.getOSName("br2_track_objects_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <Parallel success_threshold="1" failure_threshold="1">
            <TrackObjects    name="track_objects"/>
            <Move    name="move" goal="{goal}"/>
          </Parallel>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);

  geometry_msgs::msg::PoseStamped goal;
  blackboard->set("goal", goal);

  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  ASSERT_EQ(
    node_head_tracker->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  rclcpp::Rate rate(10);
  auto start = node->now();
  auto finish_tree = false;
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    finish_tree = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    rclcpp::spin_some(node);
    rate.sleep();
  }

  ASSERT_FALSE(finish_tree);
  ASSERT_EQ(
    node_head_tracker->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  while (rclcpp::ok() && !finish_tree) {
    finish_tree = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    rclcpp::spin_some(node);
    rate.sleep();
  }

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  ASSERT_EQ(
    node_head_tracker->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  finish = true;
  t.join();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
