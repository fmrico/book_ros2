cmake_minimum_required(VERSION 3.5)
project(br2_basics)

set(CMAKE_CXX_STANDARD 23)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)

add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher ${dependencies})

add_executable(publisher_class src/publisher_class.cpp)
target_link_libraries(publisher_class ${dependencies})

add_executable(subscriber_class src/subscriber_class.cpp)
target_link_libraries(subscriber_class ${dependencies})

add_executable(executors src/executors.cpp)
target_link_libraries(executors ${dependencies})

add_executable(logger src/logger.cpp)
target_link_libraries(logger ${dependencies})

add_executable(logger_class src/logger_class.cpp)
target_link_libraries(logger_class ${dependencies})

add_executable(param_reader src/param_reader.cpp)
target_link_libraries(param_reader ${dependencies})

install(TARGETS
  publisher
  publisher_class
  subscriber_class
  executors
  logger
  logger_class
  param_reader
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
