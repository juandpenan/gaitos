# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)

cmake_minimum_required(VERSION 3.20)
project(gaitos_common)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_library(gaitos_common SHARED
  src/Types.cpp
  src/Logger.cpp
  src/Params.cpp
)
target_include_directories(gaitos_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(gaitos_common
  PUBLIC Eigen3::Eigen rclcpp::rclcpp ament_index_cpp::ament_index_cpp
  PRIVATE 
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS gaitos_common EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_common test/test_common.cpp TIMEOUT 600)
  target_link_libraries(test_common gaitos_common )
  target_compile_definitions(test_common PRIVATE GAITOS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/..")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 rclcpp ament_index_cpp)
ament_package()
