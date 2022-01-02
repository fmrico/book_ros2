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


#ifndef BR2_TRACKING__PIDCONTROLLER_HPP_
#define BR2_TRACKING__PIDCONTROLLER_HPP_

#include <cmath>

namespace br2_tracking
{

class PIDController
{
public:
  PIDController(double min_ref, double max_ref, double min_output, double max_output);

  void set_pid(double n_KP, double n_KI, double n_KD);
  double get_output(double new_reference);

private:
  double KP_, KI_, KD_;

  double min_ref_, max_ref_;
  double min_output_, max_output_;
  double prev_error_, int_error_;
};

}  // namespace br2_tracking

#endif  // BR2_TRACKING__PIDCONTROLLER_HPP_
