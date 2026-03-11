cmake_minimum_required(VERSION 3.8)
project(ugv_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(ugv_teleop_core
  src/keyboard_teleop_core.cpp
  src/keyboard_teleop_common.cpp
  src/joy_axis_selector_logic.cpp
  src/joy_launcher_logic.cpp
)
target_include_directories(ugv_teleop_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ugv_teleop_core
  rclcpp
  sensor_msgs
  geometry_msgs
  rcl_interfaces
)

add_executable(joy_axis_selector_node src/joy_axis_selector_node.cpp)
target_link_libraries(joy_axis_selector_node ugv_teleop_core)
ament_target_dependencies(joy_axis_selector_node rclcpp sensor_msgs)

add_executable(joy_launcher_node src/joy_launcher_node.cpp)
target_link_libraries(joy_launcher_node ugv_teleop_core)
ament_target_dependencies(joy_launcher_node rclcpp sensor_msgs)

add_executable(keyboard_teleop_node src/keyboard_teleop_node.cpp)
target_link_libraries(keyboard_teleop_node ugv_teleop_core)
ament_target_dependencies(keyboard_teleop_node rclcpp rcl_interfaces)

add_executable(keyboard_tty_teleop_node src/keyboard_tty_teleop_node.cpp)
target_link_libraries(keyboard_tty_teleop_node ugv_teleop_core)
ament_target_dependencies(keyboard_tty_teleop_node rclcpp rcl_interfaces)

add_executable(keyboard_gui_teleop_node src/keyboard_gui_teleop_node.cpp)
target_link_libraries(keyboard_gui_teleop_node ugv_teleop_core Qt5::Widgets)
ament_target_dependencies(keyboard_gui_teleop_node rclcpp rcl_interfaces)

install(
  TARGETS
    ugv_teleop_core
    joy_axis_selector_node
    joy_launcher_node
    keyboard_teleop_node
    keyboard_tty_teleop_node
    keyboard_gui_teleop_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs geometry_msgs rcl_interfaces)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_joy_axis_selector_logic
    test/test_joy_axis_selector_logic.cpp
  )
  if(TARGET test_joy_axis_selector_logic)
    target_link_libraries(test_joy_axis_selector_logic ugv_teleop_core)
    ament_target_dependencies(test_joy_axis_selector_logic sensor_msgs)
  endif()

  ament_add_gtest(test_joy_launcher_logic
    test/test_joy_launcher_logic.cpp
  )
  if(TARGET test_joy_launcher_logic)
    target_link_libraries(test_joy_launcher_logic ugv_teleop_core)
  endif()

  ament_add_gtest(test_keyboard_teleop_core
    test/test_keyboard_teleop_core.cpp
  )
  if(TARGET test_keyboard_teleop_core)
    target_link_libraries(test_keyboard_teleop_core ugv_teleop_core)
  endif()
endif()

ament_package()
