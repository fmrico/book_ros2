#include <memory>

#include "br2_tf2_detector/ObstacleDetectorImprovedNode.hpp"
#include "br2_tf2_detector/ObstacleMonitorNode.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  auto obstacle_detector = std::make_shared<br2_tf2_detector::ObstacleDetectorImprovedNode>();
  auto obstacle_monitor = std::make_shared<br2_tf2_detector::ObstacleMonitorNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(obstacle_detector->get_node_base_interface());
  executor.add_node(obstacle_monitor->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
