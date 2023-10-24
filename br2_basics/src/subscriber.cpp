#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

void callback(const std_msgs::msg::Int32::SharedPtr msg) 
{
  RCLCPP_INFO(rclcpp::get_logger("subscriber"), "Hello %d", msg->data);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("subscriber_node");
  auto subscriber = node->create_subscription<std_msgs::msg::Int32>("int_topic", 10, &callback);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
