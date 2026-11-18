# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)

cmake_minimum_required(VERSION 3.20)
project(gaitos_system)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
set(deps gaitos_core gaitos_modes gaitos_msgs rclcpp rclcpp_lifecycle lifecycle_msgs
  geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs rosgraph_msgs tf2_ros pluginlib)
foreach(dep ${deps})
  find_package(${dep} REQUIRED)
endforeach()
find_package(Threads REQUIRED)

add_library(gaitos_system SHARED
  src/Gaitos.cpp
  src/RealTime.cpp
  src/Safety.cpp
  src/GaitosNode.cpp
)
target_include_directories(gaitos_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(gaitos_system PUBLIC
  gaitos_core::gaitos_core gaitos_modes::gaitos_modes
  ${gaitos_msgs_TARGETS} ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS} ${std_srvs_TARGETS} ${rosgraph_msgs_TARGETS} ${lifecycle_msgs_TARGETS}
  rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle tf2_ros::tf2_ros pluginlib::pluginlib
  Threads::Threads)

add_executable(gaitos_node src/gaitos_node.cpp)
target_link_libraries(gaitos_node gaitos_system)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS gaitos_system EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS gaitos_node RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(gaitos_hardware REQUIRED)
  foreach(t test_system test_safety test_plugins)
    ament_add_gtest(${t} test/${t}.cpp TIMEOUT 600)
    target_link_libraries(${t} gaitos_system gaitos_hardware::gaitos_hardware)
    target_compile_definitions(${t} PRIVATE GAITOS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/..")
  endforeach()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${deps})
ament_package()
