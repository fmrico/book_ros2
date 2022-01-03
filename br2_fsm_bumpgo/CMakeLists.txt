cmake_minimum_required(VERSION 3.5)
project(br2_fsm_bumpgo)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
)

add_executable(bumpgo_fsm src/bumpgo_fsm.cpp)
ament_target_dependencies(bumpgo_fsm ${dependencies})

install(TARGETS
bumpgo_fsm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
