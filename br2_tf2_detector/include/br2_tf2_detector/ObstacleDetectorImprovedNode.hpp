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

#ifndef BR2_TF2_DETECTOR__OBSTACLEDETECTORIMPROVEDNODE_HPP_
#define BR2_TF2_DETECTOR__OBSTACLEDETECTORIMPROVEDNODE_HPP_

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/rclcpp.hpp"

namespace br2_tf2_detector
{

class ObstacleDetectorImprovedNode : public rclcpp::Node
{
public:
  ObstacleDetectorImprovedNode();

private:
  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace br2_tf2_detector
#endif  // BR2_TF2_DETECTOR__OBSTACLEDETECTORIMPROVEDNODE_HPP_
