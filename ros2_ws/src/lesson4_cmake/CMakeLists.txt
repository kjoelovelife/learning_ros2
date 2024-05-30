cmake_minimum_required(VERSION 3.8)
project(lesson4_cmake)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
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

include_directories(include)

# add executable
## publisher_exe
add_executable(publisher_exe 
  src/publisher/callback_wall_timer_.cpp
  src/publisher/main.cpp
)

ament_target_dependencies(publisher_exe rclcpp std_msgs)

## subscriber_exe
add_executable(subscriber_exe
  src/subscriber/callback_publish_test_.cpp
  src/subscriber/main.cpp
)

ament_target_dependencies(subscriber_exe rclcpp std_msgs)


# exexutable
install(TARGETS
  publisher_exe
  subscriber_exe
  DESTINATION lib/${PROJECT_NAME}
)

# launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
