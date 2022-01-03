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

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/rclcpp.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

class BumpAndGo : public rclcpp::Node
{
public:
  BumpAndGo()
  : Node("bump_go"), state_(GOING_FORWARD)
  {
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 100);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "input_scan", 100, std::bind(&BumpAndGo::scan_callback, this, _1));
    timer_ = create_wall_timer(50ms, std::bind(&BumpAndGo::control_cycle, this));
  }

  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
  {
    last_scan_ = std::move(msg);
  }

  void control_cycle()
  {
    if (last_scan_ == nullptr) {
      return;
    }

    const auto & ranges = last_scan_->ranges;
    const float & read_front = ranges[0];
    bool obstacle_front = read_front < OBSTACLE_DISTANCE;

    geometry_msgs::msg::Twist vel;

    switch (state_) {
      case GOING_FORWARD:
        vel.linear.x = SPEED_LINEAR;
        vel.angular.z = 0.0;

        if (obstacle_front) {
          press_ts_ = now();
          state_ = GOING_BACK;
          RCLCPP_INFO(get_logger(), "GOING_FORWARD -> GOING_BACK");
        }
        break;

      case GOING_BACK:
        vel.linear.x = -SPEED_LINEAR;
        vel.angular.z = 0.0;

        if ((now() - press_ts_) > BACKING_TIME) {
          turn_ts_ = now();
          state_ = TURNING;
          RCLCPP_INFO(get_logger(), "GOING_BACK -> TURNING");
        }
        break;

      case TURNING:
        vel.linear.x = 0.0;
        vel.angular.z = SPEED_ANGULAR;

        if ((now() - turn_ts_) > TURNING_TIME) {
          state_ = GOING_FORWARD;
          RCLCPP_INFO(get_logger(), "TURNING -> GOING_FORWARD");
        }
        break;
    }

    vel_pub_->publish(vel);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;

  static const int GOING_FORWARD = 0;
  static const int GOING_BACK = 1;
  static const int TURNING = 2;

  const rclcpp::Duration TURNING_TIME {3s};
  const rclcpp::Duration BACKING_TIME {3s};

  static constexpr float SPEED_LINEAR = 0.3f;
  static constexpr float SPEED_ANGULAR = 0.5f;
  static constexpr float OBSTACLE_DISTANCE = 1.0f;

  int state_;

  rclcpp::Time press_ts_;
  rclcpp::Time turn_ts_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_bump_stop = std::make_shared<BumpAndGo>();
  rclcpp::spin(node_bump_stop->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
