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

#include "sensor_msgs/msg/laser_scan.hpp"
#include "br2_vff_avoidance/AvoidanceNode.hpp"

#include "gtest/gtest.h"


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

  visualization_msgs::msg::Marker
  make_marker_test(const std::vector<float> & vector,  br2_vff_avoidance::VFFColor vff_color)
  {
    return make_marker(vector, vff_color);
  }
};


sensor_msgs::msg::LaserScan get_scan_test_1()
{
  sensor_msgs::msg::LaserScan ret;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = M_PI / 16.0;
  ret.ranges = std::vector<float>(16, std::numeric_limits<float>::infinity());

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_2()
{
  sensor_msgs::msg::LaserScan ret;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 0.0);

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_3()
{
  sensor_msgs::msg::LaserScan ret;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[2] = 0.3;

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_4()
{
  sensor_msgs::msg::LaserScan ret;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[6] = 0.3;

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_5()
{
  sensor_msgs::msg::LaserScan ret;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[7] = 0.3;

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_6()
{
  sensor_msgs::msg::LaserScan ret;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 0.5);
  ret.ranges[7] = 0.3;

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_7()
{
  sensor_msgs::msg::LaserScan ret;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[10] = 0.3;

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_8()
{
  sensor_msgs::msg::LaserScan ret;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[14] = 0.3;

  return ret;
}

TEST(vff_tests, get_vff)
{
  auto node_avoidance = AvoidanceNodeTest();

  auto res1 = node_avoidance.get_vff_test(get_scan_test_1());
  ASSERT_EQ(res1.attractive, std::vector<float>({1.0f, 0.0f}));
  ASSERT_EQ(res1.repulsive, std::vector<float>({0.0f, 0.0f}));
  ASSERT_EQ(res1.result, std::vector<float>({1.0f, 0.0f}));

  auto res2 = node_avoidance.get_vff_test(get_scan_test_2());
  ASSERT_EQ(res2.attractive, std::vector<float>({1.0f, 0.0f}));
  ASSERT_NEAR(res2.repulsive[0], 1.0f, 0.00001f);
  ASSERT_NEAR(res2.repulsive[1], 0.0f, 0.00001f);
  ASSERT_NEAR(res2.result[0], 2.0f, 0.00001f);
  ASSERT_NEAR(res2.result[1], 0.0f, 0.00001f);

  auto res3 = node_avoidance.get_vff_test(get_scan_test_3());
  ASSERT_EQ(res3.attractive, std::vector<float>({1.0f, 0.0f}));
  ASSERT_GT(atan2(res3.repulsive[1], res3.repulsive[0]), 0.1);
  ASSERT_LT(atan2(res3.repulsive[1], res3.repulsive[0]), M_PI_2 - 0.1);
  ASSERT_GT(atan2(res3.result[1], res3.result[0]), 0.1);
  ASSERT_LT(atan2(res3.result[1], res3.result[0]), M_PI_2 - 0.1);

  auto res4 = node_avoidance.get_vff_test(get_scan_test_4());
  ASSERT_EQ(res4.attractive, std::vector<float>({1.0f, 0.0f}));
  ASSERT_GT(atan2(res4.repulsive[1], res4.repulsive[0]),  M_PI_2 + 0.1);
  ASSERT_LT(atan2(res4.repulsive[1], res4.repulsive[0]), M_PI - 0.1);
  ASSERT_GT(atan2(res4.result[1], res4.result[0]), 0.1);
  ASSERT_LT(atan2(res4.result[1], res4.result[0]), M_PI_2 - 0.1);

  std::cerr << "REP: " << res4.repulsive[0] << ", " <<  res4.repulsive[1] << std::endl;

}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
