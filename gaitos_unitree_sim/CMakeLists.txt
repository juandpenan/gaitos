# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)

cmake_minimum_required(VERSION 3.20)
project(gaitos_unitree_sim)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
set(deps gaitos_system pluginlib rclcpp std_msgs unitree_api unitree_go unitree_hg)
foreach(dep ${deps})
  find_package(${dep} REQUIRED)
endforeach()
find_package(nlohmann_json REQUIRED)

add_library(gaitos_unitree_sim SHARED
  src/Dex3Hand.cpp
  src/UnitreeEmulatorBase.cpp
  src/UnitreeHgEmulator.cpp
  src/UnitreeGoEmulator.cpp)
target_include_directories(gaitos_unitree_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(gaitos_unitree_sim PUBLIC
  gaitos_system::gaitos_system pluginlib::pluginlib rclcpp::rclcpp ${std_msgs_TARGETS}
  ${unitree_api_TARGETS} ${unitree_go_TARGETS} ${unitree_hg_TARGETS}
  PRIVATE nlohmann_json::nlohmann_json)
pluginlib_export_plugin_description_file(gaitos_system plugins.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})
install(TARGETS gaitos_unitree_sim EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(gaitos_hardware REQUIRED)
  ament_add_gtest(test_emulator test/test_emulator.cpp TIMEOUT 300)
  target_link_libraries(test_emulator gaitos_unitree_sim gaitos_hardware::gaitos_hardware)
  target_compile_definitions(test_emulator PRIVATE GAITOS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/..")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${deps})
ament_package()
