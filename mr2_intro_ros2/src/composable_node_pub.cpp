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

using namespace std::chrono_literals;

class MyNodePublisher : public rclcpp::Node
{
public:
  MyNodePublisher()
  : Node("componsable_node_pub"),
    counter(0)
  {
    pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
  }

  void doWork()
  {
    std_msgs::msg::String message;
    message.data = "Hello, world! " + std::to_string(counter++);

    RCLCPP_INFO(get_logger(), "Publishing [%s]", message.data.c_str());

    pub_->publish(message);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  int counter;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MyNodePublisher>();

  rclcpp::Rate loop_rate(500ms);
  while (rclcpp::ok()) {
    node->doWork();

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  
  rclcpp::shutdown();

  return 0;
}