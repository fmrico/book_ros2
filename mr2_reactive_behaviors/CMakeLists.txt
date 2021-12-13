cmake_minimum_required(VERSION 3.5)
project(mr2_reactive_behaviors)

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

add_executable(reactive_event_driven src/reactive_event_driven.cpp)
ament_target_dependencies(reactive_event_driven ${dependencies})

add_executable(reactive_iterative_external src/reactive_iterative_external.cpp)
ament_target_dependencies(reactive_iterative_external ${dependencies})

add_executable(reactive_iterative_internal src/reactive_iterative_internal.cpp)
ament_target_dependencies(reactive_iterative_internal ${dependencies})

install(TARGETS
  reactive_event_driven
  reactive_iterative_external
  reactive_iterative_internal
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()
