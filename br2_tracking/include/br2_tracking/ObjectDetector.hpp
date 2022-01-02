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

#ifndef BR2_TRACKING__OBJECTDETECTOR_HPP_
#define BR2_TRACKING__OBJECTDETECTOR_HPP_

#include <memory>
#include <vector>

#include "vision_msgs/msg/detection2_d.hpp"

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"

namespace br2_tracking
{

class ObjectDetector : public rclcpp::Node
{
public:
  ObjectDetector();

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

private:
  image_transport::Subscriber image_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2D>::SharedPtr detection_pub_;

  // HSV ranges for detection [h - H] [s - S] [v - V]
  std::vector<double> hsv_filter_ranges_ {0, 180, 0, 255, 0, 255};
  bool debug_ {true};
};

}  // namespace br2_tracking

#endif  // BR2_TRACKING__OBJECTDETECTOR_HPP_
