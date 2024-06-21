cmake_minimum_required(VERSION 3.8)
project(lesson11_cmake)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
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

# add executable
set(PackageName single_thread_executor)
add_executable(default_executor_exe
  src/${PackageName}/single_thread_executor.cpp
  src/${PackageName}/default_executor/main.cpp
)

ament_target_dependencies(default_executor_exe rclcpp)


set(PackageName single_thread_executor)
add_executable(single_thread_executor_exe
  src/${PackageName}/single_thread_executor.cpp
  src/${PackageName}/use_executor/main.cpp
)

ament_target_dependencies(single_thread_executor_exe rclcpp)


add_executable(single_thread_multi_node_exe
  src/${PackageName}/single_thread_executor.cpp
  src/${PackageName}/multi_node/main.cpp
)

ament_target_dependencies(single_thread_multi_node_exe rclcpp)


set(PackageName multi_thread_executor)
add_executable(multi_thread_executor_without_callback_group_exe
  src/${PackageName}/without_callback_group/multi_thread_executor.cpp
  src/${PackageName}/without_callback_group/main.cpp
)

ament_target_dependencies(multi_thread_executor_without_callback_group_exe rclcpp)

add_executable(multi_thread_executor_callback_group_ex1_exe
  src/${PackageName}/callback_group_ex1/multi_thread_executor.cpp
  src/${PackageName}/callback_group_ex1/main.cpp
)
ament_target_dependencies(multi_thread_executor_callback_group_ex1_exe rclcpp)


add_executable(multi_thread_executor_callback_group_ex2_exe
  src/${PackageName}/callback_group_ex2/multi_thread_executor.cpp
  src/${PackageName}/callback_group_ex2/main.cpp
)
ament_target_dependencies(multi_thread_executor_callback_group_ex2_exe rclcpp)



add_executable(multi_thread_executor_callback_group_ex3_exe
  src/${PackageName}/callback_group_ex3/multi_thread_executor.cpp
  src/${PackageName}/callback_group_ex3/main.cpp
)
ament_target_dependencies(multi_thread_executor_callback_group_ex3_exe rclcpp)

add_executable(multi_node_with_ex1_exe
  src/${PackageName}/multi_node/main.cpp
  src/${PackageName}/callback_group_ex1/multi_thread_executor.cpp

)
ament_target_dependencies(multi_node_with_ex1_exe rclcpp)

add_executable(multi_node_with_ex2_exe
  src/${PackageName}/multi_node/main.cpp
  src/${PackageName}/callback_group_ex2/multi_thread_executor.cpp

)
ament_target_dependencies(multi_node_with_ex2_exe rclcpp)


add_executable(multi_node_with_ex3_exe
  src/${PackageName}/multi_node/main.cpp
  src/${PackageName}/callback_group_ex3/multi_thread_executor.cpp

)
ament_target_dependencies(multi_node_with_ex3_exe rclcpp)


# install program
install(TARGETS
  default_executor_exe  
  single_thread_executor_exe
  single_thread_multi_node_exe
  multi_thread_executor_without_callback_group_exe
  multi_thread_executor_callback_group_ex1_exe
  multi_thread_executor_callback_group_ex2_exe
  multi_thread_executor_callback_group_ex3_exe
  multi_node_with_ex1_exe
  multi_node_with_ex2_exe
  multi_node_with_ex3_exe
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
