#ifndef BR2_TF2_DETECTOR__OBSTACLEDETECTORNODE_HPP_
#define BR2_TF2_DETECTOR__OBSTACLEDETECTORNODE_HPP_

#include <tf2_ros/static_transform_broadcaster.h>

#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/rclcpp.hpp"

namespace br2_tf2_detector
{

class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode();

private:
  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

}  // namespace br2_tf2_detector
#endif  // BR2_TF2_DETECTOR__OBSTACLEDETECTORNODE_HPP_
