cmake_minimum_required(VERSION 3.5)
project(br2_tracking)

set(CMAKE_CXX_STANDARD 23)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(br2_tracking_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(OpenCV REQUIRED)

set(dependencies
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  image_transport::image_transport
  cv_bridge::cv_bridge
  ${br2_tracking_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${vision_msgs_TARGETS}
  ${control_msgs_TARGETS}
)

set(export_dependencies
  rclcpp
  rclcpp_lifecycle
  br2_tracking_msgs
  sensor_msgs
  geometry_msgs
  vision_msgs
  control_msgs
  image_transport
  cv_bridge
  OpenCV
)

add_library(${PROJECT_NAME} SHARED
  src/br2_tracking/ObjectDetector.cpp
  src/br2_tracking/HeadController.cpp
  src/br2_tracking/PIDController.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} ${dependencies} ${OpenCV_LIBS})

add_executable(object_detector src/object_detector_main.cpp)
target_link_libraries(object_detector ${PROJECT_NAME} ${dependencies} ${OpenCV_LIBS})

add_executable(object_tracker src/object_tracker_main.cpp)
target_link_libraries(object_tracker ${PROJECT_NAME} ${dependencies} ${OpenCV_LIBS})


install(TARGETS
  ${PROJECT_NAME}
  object_detector
  object_tracker
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
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

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${export_dependencies})
ament_export_targets(export_${PROJECT_NAME})
ament_package()
