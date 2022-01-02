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

#include <memory>

#include "br2_tracking/ObjectDetector.hpp"
#include "br2_tracking/HeadController.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_detector = std::make_shared<br2_tracking::ObjectDetector>();
  auto node_head_controller = std::make_shared<br2_tracking::HeadController>();
  auto node_comander = rclcpp::Node::make_shared("node_commander");

  auto command_pub = node_comander->create_publisher<geometry_msgs::msg::Pose2D>("/command", 100);
  auto detection_sub = node_comander->create_subscription<vision_msgs::msg::Detection2D>(
    "/detection", rclcpp::SensorDataQoS(),
    [command_pub](vision_msgs::msg::Detection2D::SharedPtr msg) {
      geometry_msgs::msg::Pose2D command;
      command.x = (msg->bbox.center.x / msg->source_img.width) * 2.0 - 1.0;
      command.y = (msg->bbox.center.y / msg->source_img.height) * 2.0 - 1.0;
      command_pub->publish(command);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_detector);
  executor.add_node(node_head_controller);
  executor.add_node(node_comander);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
