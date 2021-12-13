cmake_minimum_required(VERSION 3.5)
project(mr2_intro_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
    rclcpp
    std_msgs
)

add_executable(simple_node src/simple_node.cpp)
ament_target_dependencies(simple_node ${dependencies})

add_executable(simple_node_pub src/simple_node_pub.cpp)
ament_target_dependencies(simple_node_pub ${dependencies})

add_executable(simple_node_sub src/simple_node_sub.cpp)
ament_target_dependencies(simple_node_sub ${dependencies})

add_executable(simple_node_pub_qos src/simple_node_pub_qos.cpp)
ament_target_dependencies(simple_node_pub_qos ${dependencies})

add_executable(simple_node_sub_qos src/simple_node_sub_qos.cpp)
ament_target_dependencies(simple_node_sub_qos ${dependencies})

add_executable(composable_node src/composable_node.cpp)
ament_target_dependencies(composable_node ${dependencies})

add_executable(composable_node_pub src/composable_node_pub.cpp)
ament_target_dependencies(composable_node_pub ${dependencies})

add_executable(composable_node_sub src/composable_node_sub.cpp)
ament_target_dependencies(composable_node_sub ${dependencies})

add_executable(composable_node_pub_timer src/composable_node_pub_timer.cpp)
ament_target_dependencies(composable_node_pub_timer ${dependencies})

add_executable(composable_node_pub_sub src/composable_node_pub_sub.cpp)
ament_target_dependencies(composable_node_pub_sub ${dependencies})

install(TARGETS
  simple_node
  simple_node_pub
  simple_node_sub
  simple_node_pub_qos
  simple_node_sub_qos
  composable_node
  composable_node_pub
  composable_node_sub
  composable_node_pub_timer
  composable_node_pub_sub
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies})

ament_package()
