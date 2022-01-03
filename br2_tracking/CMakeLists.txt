cmake_minimum_required(VERSION 3.5)
project(br2_tracking)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(OpenCV REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  vision_msgs
  control_msgs
  image_transport
  cv_bridge
  OpenCV
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/br2_tracking/ObjectDetector.cpp
  src/br2_tracking/HeadController.cpp
  src/br2_tracking/PIDController.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(object_detector src/object_detector_main.cpp)
ament_target_dependencies(object_detector ${dependencies})
target_link_libraries(object_detector ${PROJECT_NAME})

add_executable(object_tracker src/object_tracker_main.cpp)
ament_target_dependencies(object_tracker ${dependencies})
target_link_libraries(object_tracker ${PROJECT_NAME})


install(TARGETS
  ${PROJECT_NAME}
  object_detector
  object_tracker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(tests)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
