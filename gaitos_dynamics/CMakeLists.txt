# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)

cmake_minimum_required(VERSION 3.20)
project(gaitos_dynamics)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(gaitos_controllers REQUIRED)
find_package(osqp REQUIRED)

add_library(gaitos_dynamics SHARED
  src/LipmPreview.cpp
  src/LipmZmpController.cpp
  src/OsqpSolver.cpp
  src/ConvexMpc.cpp
  src/ConvexMpcController.cpp
)
target_include_directories(gaitos_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(gaitos_dynamics
  PUBLIC gaitos_controllers::gaitos_controllers
  PRIVATE osqp::osqp
)
target_compile_definitions(gaitos_dynamics PUBLIC GAITOS_WITH_OSQP)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS gaitos_dynamics EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
pluginlib_export_plugin_description_file(gaitos_core plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dynamics test/test_dynamics.cpp TIMEOUT 600)
  target_link_libraries(test_dynamics gaitos_dynamics )
  target_compile_definitions(test_dynamics PRIVATE GAITOS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/..")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(gaitos_controllers)
ament_package()
