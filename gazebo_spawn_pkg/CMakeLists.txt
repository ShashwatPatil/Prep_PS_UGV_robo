cmake_minimum_required(VERSION 3.8)
project(gazebo_spawn_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros2_control REQUIRED)
find_package(gazebo_ros2_control REQUIRED)


add_executable(spawn_robot src/spawn_robot.cpp)
ament_target_dependencies(geometry_msgs std_msgs  rclcpp gazebo_ros urdf)

target_include_directories(spawn_robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(spawn_robot PUBLIC c_std_99 cxx_std_17)

install(PROGRAMS
  src/individual_wheel_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()