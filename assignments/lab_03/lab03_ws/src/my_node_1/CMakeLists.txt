cmake_minimum_required(VERSION 3.5)
project(my_node_1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(my_node_1 src/publisher_member_function.cpp)
ament_target_dependencies(my_node_1 rclcpp std_msgs)

install(TARGETS
  my_node_1
  DESTINATION lib/${PROJECT_NAME})

ament_package()

