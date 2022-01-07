#ifndef BR2_TF2_DETECTOR__OBSTACLEMONITORNODE_HPP_
#define BR2_TF2_DETECTOR__OBSTACLEMONITORNODE_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "rclcpp/rclcpp.hpp"

namespace br2_tf2_detector
{

class ObstacleMonitorNode : public rclcpp::Node
{
public:
  ObstacleMonitorNode();

private:
  void control_cycle();
  rclcpp::TimerBase::SharedPtr timer_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

}  // namespace br2_tf2_detector
#endif  // BR2_TF2_DETECTOR__OBSTACLEMONITORNODE_HPP_
