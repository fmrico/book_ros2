
ament_add_gtest(bt_action_test bt_action_test.cpp)
target_link_libraries(bt_action_test ${dependencies})

add_executable(bt_forward bt_forward_main.cpp)
target_link_libraries(bt_forward ${dependencies})

install(TARGETS
  bt_forward
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
