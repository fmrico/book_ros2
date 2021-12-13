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

#include "BumpGo.hpp"

#include "rclcpp/rclcpp.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

class BumpAndGoImpl : public cascade_hfsm::BumpGo
{
public:
  BumpAndGoImpl()
  : BumpGo(), obstacle_front_(false)
  {
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 100);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "input_scan", 100, std::bind(&BumpAndGoImpl::scan_callback, this, _1));

    vel_pub_->on_activate();

    set_parameter({"frequency", 20.0});
  }

  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
  {
    const auto & ranges = msg->ranges;
    const float & read_front = ranges[0];
    obstacle_front_ = read_front < OBSTACLE_DISTANCE;
  }

  void Turn_code_iterative() override
  {
    geometry_msgs::msg::Twist vel;

    vel.angular.z = SPEED_ANGULAR;
    vel_pub_->publish(vel);
  }

  void Forward_code_iterative() override
  {
    geometry_msgs::msg::Twist vel;

    vel.linear.x = SPEED_LINEAR;
    vel_pub_->publish(vel);
  }

  void Back_code_iterative() override
  {
    geometry_msgs::msg::Twist vel;

    vel.linear.x = -SPEED_LINEAR;
    vel_pub_->publish(vel);
  }

  bool Forward_2_Back() override
  {
    return obstacle_front_;
  }

  bool Back_2_Turn()
  {
    return (now() - state_ts_) > BACKING_TIME;
  }

  bool Turn_2_Forward()
  {
    return (now() - state_ts_) > TURNING_TIME;
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  bool obstacle_front_;

  const rclcpp::Duration TURNING_TIME {3s};
  const rclcpp::Duration BACKING_TIME {3s};

  static constexpr float SPEED_LINEAR = 0.25f;
  static constexpr float SPEED_ANGULAR = 0.5f;
  static constexpr float OBSTACLE_DISTANCE = 0.5f;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_bump_stop = std::make_shared<BumpAndGoImpl>();

  node_bump_stop->configure();
  node_bump_stop->activate();

  rclcpp::spin(node_bump_stop->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
