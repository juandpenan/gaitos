# The messages of unitree_ros2 (BSD-3-Clause, Unitree Robotics), fetched at configure time and
# generated here without rosidl_generator_dds_idl. The package name gives the DDS type names
# (e.g. unitree_go::msg::dds_::LowState_), so it must stay the same as in unitree_ros2.
# Offline or with a local copy: -DFETCHCONTENT_SOURCE_DIR_UNITREE_ROS2=<path to unitree_ros2>
cmake_minimum_required(VERSION 3.20)
project(unitree_api)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

include(FetchContent)
set(UNITREE_ROS2_TAG 668d1ec5a05d1c38d3306bdca7d59f2ba3581a88 CACHE STRING "unitree_ros2 commit")
FetchContent_Declare(unitree_ros2
  GIT_REPOSITORY https://github.com/unitreerobotics/unitree_ros2.git
  GIT_TAG ${UNITREE_ROS2_TAG}
  SOURCE_SUBDIR no_cmake)   # only download: the msg files are used from here
FetchContent_MakeAvailable(unitree_ros2)

set(MSG_ROOT ${unitree_ros2_SOURCE_DIR}/cyclonedds_ws/src/unitree/${PROJECT_NAME})
file(GLOB MSG_FILES RELATIVE ${MSG_ROOT} CONFIGURE_DEPENDS ${MSG_ROOT}/msg/*.msg)
if(NOT MSG_FILES)
  message(FATAL_ERROR "No messages in ${MSG_ROOT}/msg")
endif()
list(TRANSFORM MSG_FILES PREPEND "${MSG_ROOT}:")   # rosidl "<base>:<relative>" tuples

rosidl_generate_interfaces(${PROJECT_NAME} ${MSG_FILES} DEPENDENCIES geometry_msgs)

install(FILES ${unitree_ros2_SOURCE_DIR}/LICENSE DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
