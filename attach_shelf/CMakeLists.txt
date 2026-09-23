cmake_minimum_required(VERSION 3.8)
project(attach_shelf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)


# Add the executable
add_executable(pre_approach_exe src/pre_approach.cpp)

ament_target_dependencies(pre_approach_exe
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
)

add_executable(approach_service src/approach_service_server.cpp)

ament_target_dependencies(approach_service
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  sensor_msgs
  std_msgs
  custom_interfaces
)


# Add the executable
add_executable(pre_approach_v2_exe src/pre_approach_v2.cpp)

ament_target_dependencies(pre_approach_v2_exe
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  custom_interfaces
)

# Install the executable
install(TARGETS
  pre_approach_exe
  pre_approach_v2_exe
  approach_service
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY 
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
