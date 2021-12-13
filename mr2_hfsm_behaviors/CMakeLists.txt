cmake_minimum_required(VERSION 3.5)
project(mr2_hfsm_behaviors)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  rclcpp_cascade_lifecycle
)

include_directories(hfsm-gen)

add_executable(bumpgo_basic src/bumpgo_basic.cpp)
ament_target_dependencies(bumpgo_basic ${dependencies})

add_executable(bumpgo_rqt_cascade
  src/bumpgo_rqt_cascade.cpp
  hfsm-gen/BumpGo.cpp
)
ament_target_dependencies(bumpgo_rqt_cascade ${dependencies})

install(TARGETS
  bumpgo_basic
  bumpgo_rqt_cascade
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()
