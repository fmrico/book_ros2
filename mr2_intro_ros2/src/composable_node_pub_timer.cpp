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

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MyNodePublisher : public rclcpp::Node
{
public:
  MyNodePublisher(const std::string & name, const std::chrono::nanoseconds & rate)
  : Node(name),
    counter_(0)
  {
    pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = create_wall_timer(
      rate, std::bind(&MyNodePublisher::timer_callback, this));
  }

  void timer_callback()
  {
    std_msgs::msg::String message;
    message.data = "Hello, world! " + std::to_string(counter_++) + " from " + get_name();

    RCLCPP_INFO(get_logger(), "Publishing [%s]", message.data.c_str());

    pub_->publish(message);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  int counter_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_A = std::make_shared<MyNodePublisher>("node_pub_A", 500ms);
  auto node_B = std::make_shared<MyNodePublisher>("node_pub_B", 1s);

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(node_A);
  executor.add_node(node_B);
 
  executor.spin();
  
  rclcpp::shutdown();

  return 0;
}