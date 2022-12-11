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

#include <limits>
#include <vector>
#include <memory>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "br2_vff_avoidance/AvoidanceNode.hpp"

#include "gtest/gtest.h"

using namespace std::chrono_literals;

class AvoidanceNodeTest : public br2_vff_avoidance::AvoidanceNode
{
public:
  br2_vff_avoidance::VFFVectors
  get_vff_test(const sensor_msgs::msg::LaserScan & scan)
  {
    return get_vff(scan);
  }

  visualization_msgs::msg::MarkerArray
  get_debug_vff_test(const br2_vff_avoidance::VFFVectors & vff_vectors)
  {
    return get_debug_vff(vff_vectors);
  }
};

sensor_msgs::msg::LaserScan get_scan_test_1(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, std::numeric_limits<float>::infinity());

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_2(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 0.0);

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_3(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[2] = 0.3;

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_4(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[6] = 0.3;

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_5(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[10] = 0.3;

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_6(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 0.5);
  ret.ranges[10] = 0.3;

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_7(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[14] = 0.3;

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_8(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[8] = 0.01;

  return ret;
}

TEST(vff_tests, get_vff)
{
  auto node_avoidance = AvoidanceNodeTest();

  rclcpp::Time ts = node_avoidance.now();

  auto res1 = node_avoidance.get_vff_test(get_scan_test_1(ts));
  ASSERT_EQ(res1.attractive, std::vector<float>({1.0f, 0.0f}));
  ASSERT_EQ(res1.repulsive, std::vector<float>({0.0f, 0.0f}));
  ASSERT_EQ(res1.result, std::vector<float>({1.0f, 0.0f}));

  auto res2 = node_avoidance.get_vff_test(get_scan_test_2(ts));
  ASSERT_EQ(res2.attractive, std::vector<float>({1.0f, 0.0f}));
  ASSERT_NEAR(res2.repulsive[0], 1.0f, 0.00001f);
  ASSERT_NEAR(res2.repulsive[1], 0.0f, 0.00001f);
  ASSERT_NEAR(res2.result[0], 2.0f, 0.00001f);
  ASSERT_NEAR(res2.result[1], 0.0f, 0.00001f);

  auto res3 = node_avoidance.get_vff_test(get_scan_test_3(ts));
  ASSERT_EQ(res3.attractive, std::vector<float>({1.0f, 0.0f}));
  ASSERT_GT(res3.repulsive[0], 0.0f);
  ASSERT_GT(res3.repulsive[1], 0.0f);
  ASSERT_GT(atan2(res3.repulsive[1], res3.repulsive[0]), 0.1);
  ASSERT_LT(atan2(res3.repulsive[1], res3.repulsive[0]), M_PI_2);
  ASSERT_GT(atan2(res3.result[1], res3.result[0]), 0.1);
  ASSERT_LT(atan2(res3.result[1], res3.result[0]), M_PI_2);

  auto res4 = node_avoidance.get_vff_test(get_scan_test_4(ts));
  ASSERT_EQ(res4.attractive, std::vector<float>({1.0f, 0.0f}));
  ASSERT_LT(res4.repulsive[0], 0.0f);
  ASSERT_GT(res4.repulsive[1], 0.0f);
  ASSERT_GT(atan2(res4.repulsive[1], res4.repulsive[0]), M_PI_2);
  ASSERT_LT(atan2(res4.repulsive[1], res4.repulsive[0]), M_PI);
  ASSERT_GT(atan2(res4.result[1], res4.result[0]), 0.0);
  ASSERT_LT(atan2(res4.result[1], res4.result[0]), M_PI_2);

  auto res5 = node_avoidance.get_vff_test(get_scan_test_5(ts));
  ASSERT_EQ(res5.attractive, std::vector<float>({1.0f, 0.0f}));
  ASSERT_LT(res5.repulsive[0], 0.0f);
  ASSERT_LT(res5.repulsive[1], 0.0f);
  ASSERT_GT(atan2(res5.repulsive[1], res5.repulsive[0]), -M_PI);
  ASSERT_LT(atan2(res5.repulsive[1], res5.repulsive[0]), -M_PI_2);
  ASSERT_LT(atan2(res5.result[1], res5.result[0]), 0.0);
  ASSERT_GT(atan2(res5.result[1], res5.result[0]), -M_PI_2);

  auto res6 = node_avoidance.get_vff_test(get_scan_test_6(ts));
  ASSERT_EQ(res6.attractive, std::vector<float>({1.0f, 0.0f}));
  ASSERT_LT(res6.repulsive[0], 0.0f);
  ASSERT_LT(res6.repulsive[1], 0.0f);
  ASSERT_GT(atan2(res6.repulsive[1], res6.repulsive[0]), -M_PI);
  ASSERT_LT(atan2(res6.repulsive[1], res6.repulsive[0]), -M_PI_2);
  ASSERT_LT(atan2(res6.result[1], res6.result[0]), 0.0);
  ASSERT_GT(atan2(res6.result[1], res6.result[0]), -M_PI_2);

  auto res7 = node_avoidance.get_vff_test(get_scan_test_7(ts));
  ASSERT_EQ(res7.attractive, std::vector<float>({1.0f, 0.0f}));
  ASSERT_GT(res7.repulsive[0], 0.0f);
  ASSERT_LT(res7.repulsive[1], 0.0f);
  ASSERT_LT(atan2(res7.repulsive[1], res7.repulsive[0]), 0.0f);
  ASSERT_GT(atan2(res7.repulsive[1], res7.repulsive[0]), -M_PI_2);
  ASSERT_LT(atan2(res7.result[1], res7.result[0]), 0.0);
  ASSERT_GT(atan2(res7.result[1], res7.result[0]), -M_PI_2);

  auto res8 = node_avoidance.get_vff_test(get_scan_test_8(ts));
  ASSERT_EQ(res8.attractive, std::vector<float>({1.0f, 0.0f}));
  ASSERT_NEAR(res8.repulsive[0], -1.0f, 0.1f);
  ASSERT_NEAR(res8.repulsive[1], 0.0f, 0.0001f);
  ASSERT_NEAR(res8.result[0], 0.0f, 0.01f);
  ASSERT_NEAR(res8.result[1], 0.0f, 0.01f);
}

TEST(vff_tests, ouput_vels)
{
  auto node_avoidance = std::make_shared<AvoidanceNodeTest>();

  // Create a testing node with a scan publisher and a speed subscriber
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto scan_pub = test_node->create_publisher<sensor_msgs::msg::LaserScan>("input_scan", 100);

  geometry_msgs::msg::Twist last_vel;
  auto vel_sub = test_node->create_subscription<geometry_msgs::msg::Twist>(
    "output_vel", 1, [&last_vel](geometry_msgs::msg::Twist::SharedPtr msg) {
      last_vel = *msg;
    });

  ASSERT_EQ(vel_sub->get_publisher_count(), 1);
  ASSERT_EQ(scan_pub->get_subscription_count(), 1);

  rclcpp::Rate rate(30);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_avoidance);
  executor.add_node(test_node);

  // Test for scan test #1
  auto start = node_avoidance->now();
  while (rclcpp::ok() && (node_avoidance->now() - start) < 1s) {
    scan_pub->publish(get_scan_test_1(node_avoidance->now()));
    executor.spin_some();
    rate.sleep();
  }
  ASSERT_NEAR(last_vel.linear.x, 0.3f, 0.0001f);
  ASSERT_NEAR(last_vel.angular.z, 0.0f, 0.0001f);

  // Test for scan test #2
  start = node_avoidance->now();
  while (rclcpp::ok() && (node_avoidance->now() - start) < 1s) {
    scan_pub->publish(get_scan_test_2(node_avoidance->now()));
    executor.spin_some();
    rate.sleep();
  }
  ASSERT_NEAR(last_vel.linear.x, 0.3f, 0.0001f);
  ASSERT_NEAR(last_vel.angular.z, 0.0f, 0.0001f);

  // Test for scan test #3
  start = node_avoidance->now();
  while (rclcpp::ok() && (node_avoidance->now() - start) < 1s) {
    scan_pub->publish(get_scan_test_3(node_avoidance->now()));
    executor.spin_some();
    rate.sleep();
  }
  ASSERT_LT(last_vel.linear.x, 0.3f);
  ASSERT_GT(last_vel.linear.x, 0.0f);
  ASSERT_GT(last_vel.angular.z, 0.0f);
  ASSERT_LT(last_vel.angular.z, M_PI_2);

  // Test for scan test #4
  start = node_avoidance->now();
  while (rclcpp::ok() && (node_avoidance->now() - start) < 1s) {
    scan_pub->publish(get_scan_test_4(node_avoidance->now()));
    executor.spin_some();
    rate.sleep();
  }
  ASSERT_LT(last_vel.linear.x, 0.3f);
  ASSERT_GT(last_vel.linear.x, 0.0f);
  ASSERT_GT(last_vel.angular.z, 0.0f);
  ASSERT_LT(last_vel.angular.z, M_PI_2);

  // Test for scan test #5
  start = node_avoidance->now();
  while (rclcpp::ok() && (node_avoidance->now() - start) < 1s) {
    scan_pub->publish(get_scan_test_5(node_avoidance->now()));
    executor.spin_some();
    rate.sleep();
  }
  ASSERT_LT(last_vel.linear.x, 0.3f);
  ASSERT_GT(last_vel.linear.x, 0.0f);
  ASSERT_LT(last_vel.angular.z, 0.0f);
  ASSERT_GT(last_vel.angular.z, -M_PI_2);

  // Test for scan test #6
  start = node_avoidance->now();
  while (rclcpp::ok() && (node_avoidance->now() - start) < 1s) {
    scan_pub->publish(get_scan_test_6(node_avoidance->now()));
    executor.spin_some();
    rate.sleep();
  }
  ASSERT_LT(last_vel.linear.x, 0.3f);
  ASSERT_GT(last_vel.linear.x, 0.0f);
  ASSERT_LT(last_vel.angular.z, 0.0f);
  ASSERT_GT(last_vel.angular.z, -M_PI_2);

  // Test for scan test #7
  start = node_avoidance->now();
  while (rclcpp::ok() && (node_avoidance->now() - start) < 1s) {
    scan_pub->publish(get_scan_test_7(node_avoidance->now()));
    executor.spin_some();
    rate.sleep();
  }
  ASSERT_LT(last_vel.linear.x, 0.3f);
  ASSERT_GT(last_vel.linear.x, 0.0f);
  ASSERT_LT(last_vel.angular.z, 0.0f);
  ASSERT_GT(last_vel.angular.z, -M_PI_2);

  // Test for scan test #8
  start = node_avoidance->now();
  while (rclcpp::ok() && (node_avoidance->now() - start) < 2s) {
    scan_pub->publish(get_scan_test_8(node_avoidance->now()));
    executor.spin_some();
    rate.sleep();
  }
  ASSERT_NEAR(last_vel.linear.x, 0.0f, 0.1f);
  ASSERT_LT(last_vel.angular.z, 0.0f);
  ASSERT_GT(last_vel.angular.z, -M_PI_2);

  // Test for stooping when scan is too old
  last_vel = geometry_msgs::msg::Twist();
  while (rclcpp::ok() && (node_avoidance->now() - start) < 3s) {
    scan_pub->publish(get_scan_test_6(start));
    executor.spin_some();
    rate.sleep();
  }
  ASSERT_NEAR(last_vel.linear.x, 0.0f, 0.01f);
  ASSERT_NEAR(last_vel.angular.z, 0.0f, 0.01f);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
