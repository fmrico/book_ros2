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

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <memory>

#include "br2_tf2_detector/ObstacleDetectorImprovedNode.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

namespace br2_tf2_detector
{

using std::placeholders::_1;
using namespace std::chrono_literals;

ObstacleDetectorImprovedNode::ObstacleDetectorImprovedNode()
: Node("obstacle_detector_improved")
{
  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS(),
    std::bind(&ObstacleDetectorImprovedNode::scan_callback, this, _1));

  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
}

void
ObstacleDetectorImprovedNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  double dist = msg->ranges[msg->ranges.size() / 2];

  if (!std::isinf(dist)) {
    tf2::Transform laser2object;
    laser2object.setOrigin(tf2::Vector3(dist, 0.0, 0.0));
    laser2object.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    geometry_msgs::msg::TransformStamped odom2laser_msg;
    tf2::Stamped<tf2::Transform> odom2laser;
    try {
      odom2laser_msg = tf_buffer_->lookupTransform(
        "odom", "base_laser_link", tf2::timeFromSec(rclcpp::Time(msg->header.stamp).seconds()),
        tf2::Duration(200ms));
      tf2::fromMsg(odom2laser_msg, odom2laser);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Obstacle transform not found: %s", ex.what());
      return;
    }

    tf2::Transform odom2object = odom2laser * laser2object;

    geometry_msgs::msg::TransformStamped odom2object_msg;
    odom2object_msg.transform = tf2::toMsg(odom2object);

    odom2object_msg.header.stamp = msg->header.stamp;
    odom2object_msg.header.frame_id = "odom";
    odom2object_msg.child_frame_id = "detected_obstacle";

    tf_broadcaster_->sendTransform(odom2object_msg);
  }
}

}  // namespace br2_tf2_detector
