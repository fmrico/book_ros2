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
#include <utility>
#include <algorithm>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "br2_vff_avoidance/AvoidanceNode.hpp"

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace br2_vff_avoidance
{

AvoidanceNode::AvoidanceNode()
: Node("avoidance_vff")
{
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 100);
  vff_debug_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("vff_debug", 100);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS(), std::bind(&AvoidanceNode::scan_callback, this, _1));

  timer_ = create_wall_timer(50ms, std::bind(&AvoidanceNode::control_cycle, this));
}

void
AvoidanceNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

void
AvoidanceNode::control_cycle()
{
  // Skip cycle if no valid recent scan available
  if (last_scan_ == nullptr || (now() - last_scan_->header.stamp) > 1s) {
    return;
  }

  // Get VFF vectors
  const VFFVectors & vff = get_vff(*last_scan_);

  // Use result vector to calculate output speed
  const auto & v = vff.result;
  double angle = atan2(v[1], v[0]);
  double module = sqrt(v[0] * v[0] + v[1] * v[1]);

  // Create ouput message, controlling speed limits
  geometry_msgs::msg::Twist vel;
  vel.linear.x = std::clamp(module, 0.0, 0.3);  // truncate linear vel to [0.0, 0.3] m/s
  vel.angular.z = std::clamp(angle, -0.5, 0.5);  // truncate rotation vel to [-0.5, 0.5] rad/s

  vel_pub_->publish(vel);

  // Produce debug information, if any interested
  if (vff_debug_pub_->get_subscription_count() > 0) {
    vff_debug_pub_->publish(get_debug_vff(vff));
  }
}

VFFVectors
AvoidanceNode::get_vff(const sensor_msgs::msg::LaserScan & scan)
{
  // This is the obstacle radious in which an obstacle affects the robot
  const float OBSTACLE_DISTANCE = 1.0;

  // Init vectors
  VFFVectors vff_vector;
  vff_vector.attractive = {OBSTACLE_DISTANCE, 0.0};  // Robot wants to go forward
  vff_vector.repulsive = {0.0, 0.0};
  vff_vector.result = {0.0, 0.0};

  // Get the index of nearest obstacle
  int min_idx = std::min_element(scan.ranges.begin(), scan.ranges.end()) - scan.ranges.begin();

  // Get the distance to nearest obstacle
  float distance_min = scan.ranges[min_idx];

  // If the obstacle is in the area that affects the robot, calculate repulsive vector
  if (distance_min < OBSTACLE_DISTANCE) {
    float angle_min = scan.angle_min + scan.angle_increment * min_idx;

    float oposite_angle = angle_min + M_PI;
    // The module of the vector is inverse to the distance to the obstacle
    float complementary_dist = OBSTACLE_DISTANCE - distance_min;

    // Get cartesian (x, y) components from polar (angle, distance)
    vff_vector.repulsive[0] = cos(oposite_angle) * complementary_dist;
    vff_vector.repulsive[1] = sin(oposite_angle) * complementary_dist;
  }

  // Calculate resulting vector adding attractive and repulsive vectors
  vff_vector.result[0] = (vff_vector.repulsive[0] + vff_vector.attractive[0]);
  vff_vector.result[1] = (vff_vector.repulsive[1] + vff_vector.attractive[1]);

  return vff_vector;
}

visualization_msgs::msg::MarkerArray
AvoidanceNode::get_debug_vff(const VFFVectors & vff_vectors)
{
  visualization_msgs::msg::MarkerArray marker_array;

  marker_array.markers.push_back(make_marker(vff_vectors.attractive, BLUE));
  marker_array.markers.push_back(make_marker(vff_vectors.repulsive, RED));
  marker_array.markers.push_back(make_marker(vff_vectors.result, GREEN));

  return marker_array;
}

visualization_msgs::msg::Marker
AvoidanceNode::make_marker(const std::vector<float> & vector, VFFColor vff_color)
{
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = "base_footprint";
  marker.header.stamp = now();
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.id = visualization_msgs::msg::Marker::ADD;

  geometry_msgs::msg::Point start;
  start.x = 0.0;
  start.y = 0.0;
  geometry_msgs::msg::Point end;
  start.x = vector[0];
  start.y = vector[1];
  marker.points = {end, start};

  marker.scale.x = 0.05;
  marker.scale.y = 0.1;

  switch (vff_color) {
    case RED:
      marker.id = 0;
      marker.color.r = 1.0;
      break;
    case GREEN:
      marker.id = 1;
      marker.color.g = 1.0;
      break;
    case BLUE:
      marker.id = 2;
      marker.color.b = 1.0;
      break;
  }
  marker.color.a = 1.0;

  return marker;
}


}  // namespace br2_vff_avoidance
