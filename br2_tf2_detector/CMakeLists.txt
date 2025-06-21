cmake_minimum_required(VERSION 3.5)
project(br2_tf2_detector)

set(CMAKE_CXX_STANDARD 23)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(dependencies
    rclcpp::rclcpp
    tf2_ros::tf2_ros
    ${geometry_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/br2_tf2_detector/ObstacleDetectorNode.cpp
  src/br2_tf2_detector/ObstacleMonitorNode.cpp
  src/br2_tf2_detector/ObstacleDetectorImprovedNode.cpp
)
target_link_libraries(${PROJECT_NAME} ${dependencies})

add_executable(detector src/detector_main.cpp)
target_link_libraries(detector ${PROJECT_NAME} ${dependencies})

add_executable(detector_improved src/detector_improved_main.cpp)
target_link_libraries(detector_improved ${PROJECT_NAME} ${dependencies})

install(TARGETS
  ${PROJECT_NAME}
  detector
  detector_improved
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
