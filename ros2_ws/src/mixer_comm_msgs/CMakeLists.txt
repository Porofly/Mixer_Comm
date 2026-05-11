cmake_minimum_required(VERSION 3.8)
project(mixer_comm_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MixerStats.msg"
  "msg/MixerPayload.msg"
  DEPENDENCIES std_msgs
)

ament_package()
