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

#ifndef BR2_REACTIVE_BEHAVIORS__AVOIDANCENODE_HPP_
#define BR2_REACTIVE_BEHAVIORS__AVOIDANCENODE_HPP_

#include <memory>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "rclcpp/rclcpp.hpp"

namespace br2_vff_avoidance
{

struct VFFVectors
{
  std::vector<float> attractive;
  std::vector<float> repulsive;
  std::vector<float> result;
};

typedef enum {RED, GREEN, BLUE, NUM_COLORS} VFFColor;

class AvoidanceNode : public rclcpp::Node
{
public:
  AvoidanceNode();

  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
  void control_cycle();

private:
  VFFVectors get_vff(const sensor_msgs::msg::LaserScan & scan);

  visualization_msgs::msg::MarkerArray get_debug_vff(const VFFVectors & vff_vectors);
  visualization_msgs::msg::Marker make_marker(
    const std::vector<float> & vector, VFFColor vff_color);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vff_debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
};

}  // namespace br2_vff_avoidance

#endif  // BR2_REACTIVE_BEHAVIORS__AVOIDANCENODE_HPP_
