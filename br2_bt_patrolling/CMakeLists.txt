cmake_minimum_required(VERSION 3.5)
project(br2_bt_patrolling)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(CMAKE_CXX_STANDARD 23)

set(dependencies
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    rclcpp_action::rclcpp_action
    behaviortree_cpp::behaviortree_cpp
    ament_index_cpp::ament_index_cpp
    ${action_msgs_TARGETS}
    ${lifecycle_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav2_msgs_TARGETS}
)

set(export_dependencies
    rclcpp
    rclcpp_lifecycle
    rclcpp_action
    behaviortree_cpp
    action_msgs
    lifecycle_msgs
    geometry_msgs
    nav2_msgs
    ament_index_cpp
)

add_library(br2_recharge_bt_node SHARED src/br2_bt_patrolling/Recharge.cpp)
add_library(br2_patrol_bt_node SHARED src/br2_bt_patrolling/Patrol.cpp)
add_library(br2_move_bt_node SHARED src/br2_bt_patrolling/Move.cpp)
add_library(br2_get_waypoint_bt_node SHARED src/br2_bt_patrolling/GetWaypoint.cpp)
add_library(br2_battery_checker_bt_node SHARED src/br2_bt_patrolling/BatteryChecker.cpp)
add_library(br2_track_objects_bt_node SHARED src/br2_bt_patrolling/TrackObjects.cpp)
list(APPEND plugin_libs
  br2_recharge_bt_node
  br2_patrol_bt_node
  br2_move_bt_node
  br2_get_waypoint_bt_node
  br2_battery_checker_bt_node
  br2_track_objects_bt_node
)

foreach(bt_plugin ${plugin_libs})
  target_include_directories(${bt_plugin} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
  target_link_libraries(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(patrolling_main src/patrolling_main.cpp)
target_link_libraries(patrolling_main ${dependencies})

install(TARGETS
  ${plugin_libs}
  patrolling_main
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY behavior_tree_xml launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # This file is vendored/third-party and may not match this repo's
  # copyright-header expectations.
  list(APPEND AMENT_LINT_AUTO_FILE_EXCLUDE "cmake/FindZMQ.cmake")

  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  add_subdirectory(tests)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${plugin_libs})
ament_export_dependencies(${export_dependencies})
ament_export_targets(export_${PROJECT_NAME})

ament_package()
