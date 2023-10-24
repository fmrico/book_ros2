// Copyright 2023 Nidhish Chadive (ru2saig)
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
#include "std_msgs/msg/int32.hpp"

void callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("subscriber"), "Hello %d", msg->data);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("subscriber_node");
  auto subscriber = node->create_subscription<std_msgs::msg::Int32>("int_topic", 10, &callback);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
