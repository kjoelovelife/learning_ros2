cmake_minimum_required(VERSION 3.8)
project(lesson12_cmake)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

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
set(PackageName qos_reliability)
add_executable(qos_reliability_subscriber_exe
  src/${PackageName}/subscriber/subscriber.cpp
  src/${PackageName}/subscriber/main.cpp
  src/${PackageName}/subscriber/callback_subscriber_.cpp
)

ament_target_dependencies(qos_reliability_subscriber_exe rclcpp std_msgs)

add_executable(qos_reliability_publisher_reliable_exe
  src/${PackageName}/publisher/publisher_reliable.cpp
  src/${PackageName}/publisher/main.cpp
  src/${PackageName}/publisher/callback_timer_.cpp
)

ament_target_dependencies(qos_reliability_publisher_reliable_exe rclcpp std_msgs)

add_executable(qos_reliability_publisher_best_effort_exe
  src/${PackageName}/publisher/publisher_best_effort.cpp
  src/${PackageName}/publisher/main.cpp
  src/${PackageName}/publisher/callback_timer_.cpp
)

ament_target_dependencies(qos_reliability_publisher_best_effort_exe rclcpp std_msgs)


# add executable
set(PackageName dds_example)
add_executable(dds_example_subscriber_exe
  src/${PackageName}/subscriber/subscriber.cpp
  src/${PackageName}/subscriber/main.cpp
  src/${PackageName}/subscriber/callback_subscriber_.cpp
  src/${PackageName}/subscriber/process_dds_test_data_.cpp
)
ament_target_dependencies(dds_example_subscriber_exe rclcpp std_msgs)

add_executable(dds_example_publisher_exe
  src/${PackageName}/publisher/publisher.cpp
  src/${PackageName}/publisher/main.cpp
  src/${PackageName}/publisher/callback_timer_.cpp
)
ament_target_dependencies(dds_example_publisher_exe rclcpp std_msgs)


# add executable
set(PackageName lifecycle_example)
add_executable(managed_scan_exe
  src/${PackageName}/managed_scan/main.cpp
  src/${PackageName}/managed_scan/on_activate.cpp
  src/${PackageName}/managed_scan/on_cleanup.cpp
  src/${PackageName}/managed_scan/on_configure.cpp
  src/${PackageName}/managed_scan/on_deactivate.cpp
  src/${PackageName}/managed_scan/on_error.cpp
  src/${PackageName}/managed_scan/on_shutdown.cpp
  src/${PackageName}/managed_scan/publish.cpp
)
ament_target_dependencies(managed_scan_exe rclcpp std_msgs lifecycle_msgs rclcpp_lifecycle)




# install program
install(TARGETS
  qos_reliability_subscriber_exe
  qos_reliability_publisher_reliable_exe
  qos_reliability_publisher_best_effort_exe 
  dds_example_subscriber_exe
  dds_example_publisher_exe
  managed_scan_exe
  DESTINATION lib/${PROJECT_NAME}
)

# install directory launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
