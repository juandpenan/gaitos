# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)

cmake_minimum_required(VERSION 3.20)
project(gaitos_sensors)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(gaitos_core REQUIRED)

add_library(gaitos_sensors SHARED
  src/Sensors.cpp
)
target_include_directories(gaitos_sensors PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(gaitos_sensors
  PUBLIC gaitos_core::gaitos_core
  PRIVATE 
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS gaitos_sensors EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
pluginlib_export_plugin_description_file(gaitos_core plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(gaitos_core)
ament_package()
