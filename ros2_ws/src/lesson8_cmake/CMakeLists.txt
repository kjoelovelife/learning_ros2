cmake_minimum_required(VERSION 3.8)
project(lesson8_cmake)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lesson_interfaces REQUIRED)
find_package(turtlesim REQUIRED)

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

# add executable
set(PackageName turtle_spawner)
add_executable(turtle_spawner_exe
  src/${PackageName}/turtle_spawner.cpp
  src/${PackageName}/call_spawn_service_.cpp
  src/${PackageName}/main.cpp
  src/${PackageName}/callback_spawn_turtle_timer_.cpp
  src/${PackageName}/publish_alive_turtles_.cpp
  src/${PackageName}/callback_catch_turtle_.cpp
  src/${PackageName}/call_kill_turtle_service_.cpp
)

ament_target_dependencies(turtle_spawner_exe rclcpp geometry_msgs lesson_interfaces turtlesim)

set(PackageName follow_the_turtle)
add_executable(follow_the_turtle_exe
  src/${PackageName}/follow_the_turtle.cpp
  src/${PackageName}/main.cpp
  src/${PackageName}/callback_control_loop_.cpp
  src/${PackageName}/callback_turtle1_pose_.cpp
  src/${PackageName}/callback_turtle2_pose_.cpp
  src/${PackageName}/call_turtlesim_spawn_service_.cpp
)

ament_target_dependencies(follow_the_turtle_exe rclcpp geometry_msgs turtlesim)


set(PackageName catch_the_turtle)
add_executable(catch_the_turtle_exe
  src/${PackageName}/catch_the_turtle.cpp
  src/${PackageName}/main.cpp
  src/${PackageName}/controller_loop_.cpp
  src/${PackageName}/callback_turtle_pose_.cpp
  src/${PackageName}/callback_alive_turtles_.cpp
  src/${PackageName}/call_catch_turtle_service_.cpp
)

ament_target_dependencies(catch_the_turtle_exe rclcpp geometry_msgs turtlesim lesson_interfaces)


# install program
install(TARGETS
  turtle_spawner_exe
  follow_the_turtle_exe
  catch_the_turtle_exe
  DESTINATION lib/${PROJECT_NAME}
)

# install directory launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
