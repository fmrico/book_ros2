// Copyright 2024 Intelligent Robotics Lab
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


#include "yaets/tracing.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  yaets::TraceSession session("wakeup.log");

  // sched_param sch;
  // sch.sched_priority = 90;
  // if (sched_setscheduler(0, SCHED_FIFO, &sch) == -1) {
  //   throw std::runtime_error{std::string("failed to set scheduler: ") + std::strerror(errno)};
  // }

  rclcpp::Rate loop_rate(20ms);
  while (rclcpp::ok()) {
    {
      TRACE_EVENT(session);
    }
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
