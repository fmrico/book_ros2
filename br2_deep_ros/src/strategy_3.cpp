// Copyright 2024 Intelligent Robotics Lab
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

#include <fstream>

#include "yaets/tracing.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection3_d.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


yaets::TraceSession session("strategy_3.log");


void waste_time(rclcpp::Node::SharedPtr node, const rclcpp::Duration & duration)
{
  auto start = node->now();
  while (node->now() - start < duration);
}


class SensorDriverNode : public rclcpp::Node
{
public:
  SensorDriverNode() : Node("sensor_driver")
  {
    rt_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
  
    pub_ = create_publisher<sensor_msgs::msg::Image>("image", 100);
    timer_scan_ = create_wall_timer(
      10ms, std::bind(&SensorDriverNode::produce_data, this), rt_callback_group_);
    timer_state_ = create_wall_timer(100ms, std::bind(&SensorDriverNode::print_state, this));
  }

  void produce_data()
  {
    SHARED_TRACE_START("brake_process");

    waste_time(shared_from_this(), 200us);

    sensor_msgs::msg::Image image_msg;
    pub_->publish(image_msg);
  }

  void print_state()
  {
    waste_time(shared_from_this(), 1ms);
  }

  rclcpp::CallbackGroup::SharedPtr get_rt_callback_group()
  {
    return rt_callback_group_;
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_scan_, timer_state_;
  rclcpp::CallbackGroup::SharedPtr rt_callback_group_;
};


class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode() : Node("obstacle_detector")
  {
    rt_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = rt_callback_group_;
  
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "image", 100,
      std::bind(&ObstacleDetectorNode::detect_obstacle, this, _1),
      sub_options);
    pub_ = create_publisher<vision_msgs::msg::Detection3D>("obstacles", 100);
    timer_state_ = create_wall_timer(100ms, std::bind(&ObstacleDetectorNode::print_state, this));
  }

  void detect_obstacle(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    waste_time(shared_from_this(), 5ms);
    
    vision_msgs::msg::Detection3D detection_msg;
    pub_->publish(detection_msg);
  }


  void print_state()
  {
    waste_time(shared_from_this(), 1ms);
  }

  rclcpp::CallbackGroup::SharedPtr get_rt_callback_group()
  {
    return rt_callback_group_;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher< vision_msgs::msg::Detection3D>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_state_;
  rclcpp::CallbackGroup::SharedPtr rt_callback_group_;
};


class LoggerNode : public rclcpp::Node
{
public:
  LoggerNode() : Node("logger_node")
  {
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "image", 100, std::bind(&LoggerNode::cb, this, _1));
 
    timer_state_ = create_wall_timer(10ms, std::bind(&LoggerNode::print_state, this));
  }

  void cb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    waste_time(shared_from_this(), 500us);
  }


  void print_state()
  {
    waste_time(shared_from_this(), 2ms);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_state_;
};


class BrakeActuatorNode : public rclcpp::Node
{
public:
  BrakeActuatorNode() : Node("brake_actuator")
  {
    rt_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = rt_callback_group_;
  
    sub_ = create_subscription<vision_msgs::msg::Detection3D>(
      "obstacles", 100,
      std::bind(&BrakeActuatorNode::react_obstacle, this, _1),
      sub_options);
    timer_state_ = create_wall_timer(100ms, std::bind(&BrakeActuatorNode::print_state, this));
  }

  void react_obstacle(vision_msgs::msg::Detection3D::SharedPtr msg)
  {
    waste_time(shared_from_this(), 2ms);
    SHARED_TRACE_END("brake_process");
  }


  void print_state()
  {
    waste_time(shared_from_this(), 1ms);
  }

  rclcpp::CallbackGroup::SharedPtr get_rt_callback_group()
  {
    return rt_callback_group_;
  }

private:
  rclcpp::Subscription<vision_msgs::msg::Detection3D>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_state_;
  rclcpp::CallbackGroup::SharedPtr rt_callback_group_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  SHARED_TRACE_INIT(session, "brake_process");

  auto node_sensor_driver = std::make_shared<SensorDriverNode>();
  auto node_obstacle_detector = std::make_shared<ObstacleDetectorNode>();
  auto node_logger = std::make_shared<LoggerNode>();
  auto node_brake_actuator = std::make_shared<BrakeActuatorNode>();

  rclcpp::executors::SingleThreadedExecutor no_rt_executor;
  rclcpp::executors::MultiThreadedExecutor rt_executor(rclcpp::ExecutorOptions(), 3);

  no_rt_executor.add_node(node_sensor_driver);
  no_rt_executor.add_node(node_obstacle_detector);
  no_rt_executor.add_node(node_logger);
  no_rt_executor.add_node(node_brake_actuator);

  rt_executor.add_callback_group(
    node_sensor_driver->get_rt_callback_group(),
    node_sensor_driver->get_node_base_interface());
  rt_executor.add_callback_group(
    node_obstacle_detector->get_rt_callback_group(),
    node_obstacle_detector->get_node_base_interface());
  rt_executor.add_callback_group(
    node_brake_actuator->get_rt_callback_group(),
    node_brake_actuator->get_node_base_interface());

  auto rt_thread = std::thread(
    [&]() {
      // sched_param sch;
      // sch.sched_priority = 90;
      // 
      // if (sched_setscheduler(0, SCHED_FIFO, &sch) == -1) {
      //   throw std::runtime_error{std::string("failed to set scheduler: ") + std::strerror(errno)};
      // }
      
      rt_executor.spin();
  });

  no_rt_executor.spin();

  rt_thread.join();

  rclcpp::shutdown();
  return 0;
}
