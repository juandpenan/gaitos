# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)

cmake_minimum_required(VERSION 3.20)
project(gaitos_naoqi_sim)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
set(deps gaitos_system geometry_msgs naoqi_bridge_msgs nav_msgs pluginlib rclcpp sensor_msgs
  std_msgs tf2_ros)
foreach(dep ${deps})
  find_package(${dep} REQUIRED)
endforeach()

add_library(gaitos_naoqi_sim SHARED src/NaoqiDriverEmulator.cpp)
target_include_directories(gaitos_naoqi_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(gaitos_naoqi_sim PUBLIC
  gaitos_system::gaitos_system pluginlib::pluginlib rclcpp::rclcpp tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS} ${naoqi_bridge_msgs_TARGETS} ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS} ${std_msgs_TARGETS})
pluginlib_export_plugin_description_file(gaitos_system plugins.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})
install(TARGETS gaitos_naoqi_sim EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(gaitos_hardware REQUIRED)
  ament_add_gtest(test_naoqi_emulator test/test_naoqi_emulator.cpp TIMEOUT 300)
  target_link_libraries(test_naoqi_emulator gaitos_naoqi_sim gaitos_hardware::gaitos_hardware)
  target_compile_definitions(test_naoqi_emulator PRIVATE GAITOS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/..")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${deps})
ament_package()
