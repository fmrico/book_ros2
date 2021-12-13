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

class MyNodeSubscriber : public rclcpp::Node
{
public:
  MyNodeSubscriber(const std::string & name)
  : Node(name)
  {
    sub_ = create_subscription<std_msgs::msg::String>(
      "chatter", 10, std::bind(&MyNodeSubscriber::callback, this, _1));
  }

private:
  void callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: [%s] in %s",
      msg->data.c_str(), get_name());
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_A = std::make_shared<MyNodeSubscriber>("node_A");
  auto node_B = std::make_shared<MyNodeSubscriber>("node_B");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_A);
  executor.add_node(node_B);
 
  executor.spin();

  rclcpp::shutdown();

  return 0;
}