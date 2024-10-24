cmake_minimum_required(VERSION 3.8)
project(text_to_cmd_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(topic src/topic.cpp)
add_executable(sub src/text_to_cmd.cpp)

ament_target_dependencies(topic rclcpp std_msgs geometry_msgs)
ament_target_dependencies(sub rclcpp std_msgs geometry_msgs)

install(TARGETS
  topic
  sub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
