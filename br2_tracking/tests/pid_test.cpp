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

#include <random>

#include "br2_tracking/PIDController.hpp"

#include "gtest/gtest.h"

TEST(pid_tests, pid_test_1)
{
  br2_tracking::PIDController pid(-1.0, 1.0, -1.0, 1.0);

  ASSERT_NEAR(pid.get_output(0.0), 0.0, 0.05);
  ASSERT_LT(pid.get_output(0.1), -0.7);
  ASSERT_GT(pid.get_output(0.1), -0.4);
  ASSERT_LT(pid.get_output(0.1), 0.3);
}

TEST(pid_tests, pid_test_2)
{
  br2_tracking::PIDController pid(-1.0, 1.0, -1.0, 1.0);
  pid.set_pid(1.0, 0.0, 0.0);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-5.0, 5.0);

  for (int n = 0; n < 100000; n++) {
    double random_number = dis(gen);
    double output = pid.get_output(random_number);

    ASSERT_LE(output, 1.0);
    ASSERT_GE(output, -1.0);

    if (output < -2.0) {
      ASSERT_NEAR(output, -1.0, 0.01);
    }
    if (output > 2.0) {
      ASSERT_NEAR(output, 1.0, 0.01);
    }
    if (output > 0.0) {
      ASSERT_GT(output, 0.0);
    }
    if (output < 0.0) {
      ASSERT_LT(output, 0.0);
    }
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
