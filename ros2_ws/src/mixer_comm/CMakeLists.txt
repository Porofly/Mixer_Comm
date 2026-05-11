cmake_minimum_required(VERSION 3.8)
project(mixer_comm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mixer_comm_msgs REQUIRED)

add_executable(mixer_serial_node
  src/mixer_serial_node.cpp
  src/serial_port.cpp
  src/frame_codec.cpp
)
target_include_directories(mixer_serial_node PRIVATE include)
ament_target_dependencies(mixer_serial_node
  rclcpp
  std_msgs
  mixer_comm_msgs
)

install(TARGETS mixer_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
