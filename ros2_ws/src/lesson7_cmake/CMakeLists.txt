cmake_minimum_required(VERSION 3.8)
project(lesson7_cmake)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

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

# include 
include_directories(include)

add_executable(teleop_in_terminal_exe
  src/teleop_in_terminal.cpp
  src/main.cpp
  src/get_key_.cpp
  src/backward_.cpp
  src/forward_.cpp
  src/turn_left_.cpp
  src/turn_right_.cpp
  src/stop_.cpp
)

ament_target_dependencies(teleop_in_terminal_exe rclcpp geometry_msgs)

# install program
install(TARGETS
  teleop_in_terminal_exe
  DESTINATION lib/${PROJECT_NAME}
)

# install directory launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
