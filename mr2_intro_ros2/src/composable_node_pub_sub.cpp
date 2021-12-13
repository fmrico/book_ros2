// Copyright 2020 Intelligent Robotics Lab
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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class MyNodePublisher : public rclcpp::Node
{
public:
  MyNodePublisher(const std::string & name, const std::chrono::nanoseconds & rate)
  : Node(name),
    counter(0)
  {
    pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = create_wall_timer(
      rate, std::bind(&MyNodePublisher::timer_callback, this));
  }

  void timer_callback()
  {
    std_msgs::msg::String message;
    message.data = "Hello, world! " + std::to_string(counter++) + " from " + get_name();

    RCLCPP_INFO(get_logger(), "Publishing [%s]", message.data.c_str());

    pub_->publish(message);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  int counter;
};

void finite_worker() 
{
  auto node = rclcpp::Node::make_shared("worker_node");
  int counter = 0;
  auto subscription = node->create_subscription<std_msgs::msg::String>(
    "chatter", 10, [&counter, &node](std_msgs::msg::String::UniquePtr msg) {
        RCLCPP_INFO(node->get_logger(), "I heard: '%s' with counter %d",
        msg->data.c_str(), counter++);
      });

  rclcpp::Rate loop_rate(100ms);
  while (rclcpp::ok() && counter < 10) {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_A = std::make_shared<MyNodePublisher>("node_A", 500ms);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_A);
  
  std::shared_future<void> finite_worker_future = std::async(std::launch::async,
    std::bind(finite_worker));

  executor.spin_until_future_complete(finite_worker_future);

  rclcpp::shutdown();

  return 0;
}