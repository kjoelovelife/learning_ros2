cmake_minimum_required(VERSION 3.8)
project(lesson6_cmake)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lesson_interfaces REQUIRED)

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

set(ProjectName move_turtlesim_server)

# add executable
## move_turtlesim_server_exe
add_executable(move_turtlesim_server_exe
  src/${ProjectName}/move_turtlesim_server.cpp
  src/${ProjectName}/callback_service_.cpp
  src/${ProjectName}/main.cpp
  src/${ProjectName}/move_circle_.cpp
  src/${ProjectName}/move_line_.cpp
  src/${ProjectName}/move_square_.cpp
  src/${ProjectName}/move_triangle_.cpp
  src/${ProjectName}/stop_.cpp
)

ament_target_dependencies(move_turtlesim_server_exe rclcpp geometry_msgs lesson_interfaces)


set(ProjectName move_turtlesim_client)
## move_turtlesim_client_exe
add_executable(move_turtlesim_client_exe
  src/${ProjectName}/move_turtlesim_client.cpp
  src/${ProjectName}/main.cpp
)

ament_target_dependencies(move_turtlesim_client_exe rclcpp lesson_interfaces)

# install program
install(TARGETS
  move_turtlesim_server_exe
  move_turtlesim_client_exe
  DESTINATION lib/${PROJECT_NAME}
)

# install directory launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
