cmake_minimum_required(VERSION 3.5)
project(br2_deep_ros)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(yaets REQUIRED)

set(export_dependencies
  rclcpp
  std_msgs
  sensor_msgs
  vision_msgs
  yaets
)

set(link_dependencies
  rclcpp::rclcpp
  yaets::yaets
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${vision_msgs_TARGETS}
)

add_executable(executors src/executors.cpp)
target_link_libraries(executors ${link_dependencies})

add_executable(executors_cbg src/executors_cbg.cpp)
target_link_libraries(executors_cbg ${link_dependencies})

add_executable(executors_cbg_reentrant src/executors_cbg_reentrant.cpp)
target_link_libraries(executors_cbg_reentrant ${link_dependencies})

add_executable(wakeup src/wakeup.cpp)
target_link_libraries(wakeup ${link_dependencies})

add_executable(wakeup_ros src/wakeup_ros.cpp)
target_link_libraries(wakeup_ros ${link_dependencies})

add_executable(strategy_1 src/strategy_1.cpp)
target_link_libraries(strategy_1 ${link_dependencies})

add_executable(strategy_2 src/strategy_2.cpp)
target_link_libraries(strategy_2 ${link_dependencies})

add_executable(strategy_3 src/strategy_3.cpp)
target_link_libraries(strategy_3 ${link_dependencies})

install(TARGETS
  executors
  executors_cbg
  executors_cbg_reentrant
  wakeup
  wakeup_ros
  strategy_1
  strategy_2
  strategy_3
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${export_dependencies})
ament_package()
