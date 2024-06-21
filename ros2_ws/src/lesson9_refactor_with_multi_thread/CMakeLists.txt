cmake_minimum_required(VERSION 3.8)
project(lesson9_refactor_with_multi_thread)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
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
set(PackageName turtle_draw_circle_with_parameter)
add_executable(turtle_draw_circle_with_parameter_exe
  src/${PackageName}/turtle_draw_circle_with_parameter.cpp
  src/${PackageName}/main.cpp
  src/${PackageName}/callback_timer_.cpp
  src/${PackageName}/set_speed_.cpp
  src/${PackageName}/declare_ros2_parameter_.cpp
)

ament_target_dependencies(turtle_draw_circle_with_parameter_exe rclcpp geometry_msgs rcl_interfaces )

# add executable
set(PackageName get_turtlesim_background)
add_executable(get_turtlesim_background_exe
  src/${PackageName}/get_turtlesim_background.cpp
  src/${PackageName}/main.cpp
  src/${PackageName}/callback_timer_.cpp
  src/${PackageName}/call_get_parameter_service_.cpp
  # src/${PackageName}/customic_create_client_.cpp
)

ament_target_dependencies(get_turtlesim_background_exe rclcpp rcl_interfaces)

# add executable
set(PackageName set_turtle_velocity)
add_executable(set_turtle_velocity_exe
  src/${PackageName}/set_turtle_velocity.cpp
  src/${PackageName}/main.cpp
  src/${PackageName}/callback_timer_.cpp
  src/${PackageName}/set_velocity_bound_.cpp
  src/${PackageName}/call_describe_parameter_service_.cpp
  src/${PackageName}/call_set_parameter_atomically_service_.cpp
)

ament_target_dependencies(set_turtle_velocity_exe rclcpp rcl_interfaces)

# add executable
set(PackageName final_exam)
add_executable(final_exam_exe
  src/${PackageName}/final_exam.cpp
  src/${PackageName}/main.cpp
  src/${PackageName}/callback_timer_.cpp
  src/${PackageName}/call_get_describe_parameter_service_.cpp
  src/${PackageName}/call_get_parameters_service_.cpp
  src/${PackageName}/call_set_parameter_atomically_service_.cpp
)

ament_target_dependencies(final_exam_exe rclcpp rcl_interfaces)


# install program
install(TARGETS
  turtle_draw_circle_with_parameter_exe
  get_turtlesim_background_exe
  set_turtle_velocity_exe
  final_exam_exe
  DESTINATION lib/${PROJECT_NAME}
)

# install directory launch
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
