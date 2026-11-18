# The interfaces of naoqi_driver (ros-naoqi naoqi_bridge_msgs2, Apache 2.0), fetched at configure
# time and generated here. Offline: -DFETCHCONTENT_SOURCE_DIR_NAOQI_BRIDGE_MSGS2=<path>
cmake_minimum_required(VERSION 3.20)
project(naoqi_bridge_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
set(deps action_msgs geometry_msgs nav_msgs sensor_msgs std_msgs trajectory_msgs)
foreach(dep ${deps})
  find_package(${dep} REQUIRED)
endforeach()

include(FetchContent)
set(NAOQI_BRIDGE_MSGS2_TAG 4e7a0fc9a7718c87ba1e3b3b7bf25361c37da2ed CACHE STRING "naoqi_bridge_msgs2 commit")
FetchContent_Declare(naoqi_bridge_msgs2
  GIT_REPOSITORY https://github.com/ros-naoqi/naoqi_bridge_msgs2.git
  GIT_TAG ${NAOQI_BRIDGE_MSGS2_TAG}
  SOURCE_SUBDIR no_cmake)   # only download: its files are generated here
FetchContent_MakeAvailable(naoqi_bridge_msgs2)

set(ROOT ${naoqi_bridge_msgs2_SOURCE_DIR})
file(GLOB FILES RELATIVE ${ROOT} CONFIGURE_DEPENDS
  ${ROOT}/msg/*.msg ${ROOT}/srv/*.srv ${ROOT}/action/*.action)
if(NOT FILES)
  message(FATAL_ERROR "No interfaces in ${ROOT}")
endif()
list(TRANSFORM FILES PREPEND "${ROOT}:")   # rosidl "<base>:<relative>" tuples
rosidl_generate_interfaces(${PROJECT_NAME} ${FILES} DEPENDENCIES ${deps})

install(FILES ${ROOT}/LICENSE DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
