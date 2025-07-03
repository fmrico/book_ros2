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
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


yaets::TraceSession session("strategy_1.log");


void waste_time(rclcpp::Node::SharedPtr node, const rclcpp::Duration & duration)
{
  auto start = node->now();
  while (node->now() - start < duration) {}
}

class ProducerNode : public rclcpp::Node
{
public:
  ProducerNode()
  : Node("producer_node")
  {
    pub_ = create_publisher<std_msgs::msg::Int32>("int_topic", 100);
    timer_ = create_wall_timer(10ms, std::bind(&ProducerNode::timer_callback, this));
  }

  void timer_callback()
  {
    TRACE_EVENT(session);

    waste_time(shared_from_this(), 200us);

    message_.data += 1;
    pub_->publish(message_);
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std_msgs::msg::Int32 message_;
};

class ConsumerNode : public rclcpp::Node
{
public:
  ConsumerNode()
  : Node("consumer_node")
  {
    sub_ = create_subscription<std_msgs::msg::Int32>(
      "int_topic", 100, std::bind(&ConsumerNode::cb, this, _1));

    timer_ = create_wall_timer(10ms, std::bind(&ConsumerNode::timer_cb, this));
  }

  void cb(const std_msgs::msg::Int32::SharedPtr msg)
  {
    TRACE_EVENT(session);

    waste_time(shared_from_this(), 500us);
  }


  void timer_cb()
  {
    TRACE_EVENT(session);

    waste_time(shared_from_this(), 2ms);
  }

private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class LoggerNode : public rclcpp::Node
{
public:
  LoggerNode()
  : Node("logger_node")
  {
    sub_ = create_subscription<std_msgs::msg::Int32>(
      "int_topic", 100, std::bind(&LoggerNode::cb, this, _1));

    timer_ = create_wall_timer(10ms, std::bind(&LoggerNode::timer_cb, this));
  }

  void cb(const std_msgs::msg::Int32::SharedPtr msg)
  {
    TRACE_EVENT(session);

    waste_time(shared_from_this(), 500us);
  }


  void timer_cb()
  {
    TRACE_EVENT(session);

    waste_time(shared_from_this(), 2ms);
  }

private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_producer = std::make_shared<ProducerNode>();
  auto node_consumer = std::make_shared<ConsumerNode>();
  auto node_logger = std::make_shared<LoggerNode>();

  rclcpp::executors::SingleThreadedExecutor no_rt_executor;
  rclcpp::executors::SingleThreadedExecutor rt_executor;

  no_rt_executor.add_node(node_producer);
  no_rt_executor.add_node(node_logger);
  rt_executor.add_node(node_consumer);

  auto rt_thread = std::thread(
    [&]() {
      sched_param sch;
      sch.sched_priority = 90;

      if (sched_setscheduler(0, SCHED_FIFO, &sch) == -1) {
        throw std::runtime_error{std::string("failed to set scheduler: ") + std::strerror(errno)};
      }

      rt_executor.spin();
  });

  no_rt_executor.spin();

  rt_thread.join();

  rclcpp::shutdown();
  return 0;
}
