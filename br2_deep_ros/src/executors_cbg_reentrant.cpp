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


yaets::TraceSession session("session1.log");

class ProducerNode : public rclcpp::Node
{
public:
  ProducerNode()
  : Node("producer_node")
  {
    pub_1_ = create_publisher<std_msgs::msg::Int32>("topic_1", 100);
    pub_2_ = create_publisher<std_msgs::msg::Int32>("topic_2", 100);
    timer_ = create_wall_timer(1ms, std::bind(&ProducerNode::timer_callback, this));
  }

  void timer_callback()
  {
    message_.data += 1;
    pub_1_->publish(message_);
    message_.data += 1;
    pub_2_->publish(message_);
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_1_, pub_2_;
  rclcpp::TimerBase::SharedPtr timer_;
  std_msgs::msg::Int32 message_;
};

class ConsumerNode : public rclcpp::Node
{
public:
  ConsumerNode()
  : Node("consumer_node")
  {
    custom_cb_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions options;
    options.callback_group = custom_cb_;

    sub_2_ = create_subscription<std_msgs::msg::Int32>("topic_2", 100,
      std::bind(&ConsumerNode::cb_2, this, _1), options);
    sub_1_ = create_subscription<std_msgs::msg::Int32>("topic_1", 100,
      std::bind(&ConsumerNode::cb_1, this, _1), options);

    timer_ = create_wall_timer(10ms, std::bind(&ConsumerNode::timer_cb, this), custom_cb_);
  }

  void cb_1(const std_msgs::msg::Int32::SharedPtr msg)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    TRACE_EVENT(session);

    waste_time(500us);
  }

  void cb_2(const std_msgs::msg::Int32::SharedPtr msg)
  {
    TRACE_EVENT(session);

    waste_time(500us);
  }

  void timer_cb()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    TRACE_EVENT(session);

    waste_time(5ms);
  }

  void waste_time(const rclcpp::Duration & duration)
  {
    auto start = now();
    while (now() - start < duration) {}
  }

private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_1_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_2_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr custom_cb_;
  std::mutex mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_pub = std::make_shared<ProducerNode>();
  auto node_sub1 = std::make_shared<ConsumerNode>();

  // rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 8);

  executor.add_node(node_pub);
  executor.add_node(node_sub1);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
