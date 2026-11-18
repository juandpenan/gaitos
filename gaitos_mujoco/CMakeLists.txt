# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)

cmake_minimum_required(VERSION 3.20)
project(gaitos_mujoco)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(gaitos_core REQUIRED)
find_package(gaitos_system REQUIRED)
find_package(mujoco REQUIRED)

add_library(gaitos_mujoco SHARED src/MujocoHardware.cpp)
target_include_directories(gaitos_mujoco PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(gaitos_mujoco PUBLIC gaitos_core::gaitos_core mujoco::mujoco)
pluginlib_export_plugin_description_file(gaitos_core plugins.xml)

# Offscreen snapshots (EGL) of scripted runs, for documentation images
find_package(OpenGL QUIET COMPONENTS EGL OpenGL)
find_package(ZLIB QUIET)
if(OpenGL_EGL_FOUND AND ZLIB_FOUND)
  add_executable(snapshot tools/snapshot.cpp)
  target_link_libraries(snapshot gaitos_mujoco gaitos_system::gaitos_system OpenGL::EGL ZLIB::ZLIB)
  install(TARGETS snapshot RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})
install(TARGETS gaitos_mujoco EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  ament_add_gtest(test_mujoco test/test_mujoco.cpp TIMEOUT 900)
  target_link_libraries(test_mujoco gaitos_mujoco gaitos_system::gaitos_system ament_index_cpp::ament_index_cpp)
  target_compile_definitions(test_mujoco PRIVATE GAITOS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/..")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(gaitos_core mujoco)
ament_package()
