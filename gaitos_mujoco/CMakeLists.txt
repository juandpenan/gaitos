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

# Renderer node extension: simulated cameras as ROS images (EGL, headless) and a viewer
# window (GLFW, optional)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(glfw3 QUIET)
if(OpenGL_EGL_FOUND)
  add_library(gaitos_mujoco_renderer SHARED src/MujocoRenderer.cpp)
  target_link_libraries(gaitos_mujoco_renderer PUBLIC gaitos_mujoco gaitos_system::gaitos_system
    rclcpp_lifecycle::rclcpp_lifecycle pluginlib::pluginlib ${sensor_msgs_TARGETS}
    ${tf2_msgs_TARGETS} OpenGL::EGL OpenGL::OpenGL)
  if(glfw3_FOUND)
    target_link_libraries(gaitos_mujoco_renderer PRIVATE glfw)
    target_compile_definitions(gaitos_mujoco_renderer PRIVATE GAITOS_MUJOCO_GLFW)
  else()
    message(WARNING "GLFW not found (libglfw3-dev): MujocoRenderer without viewer window")
  endif()
  pluginlib_export_plugin_description_file(gaitos_system extensions.xml)
  install(TARGETS gaitos_mujoco_renderer EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
else()
  message(WARNING "EGL not found: no MujocoRenderer (simulated cameras, viewer)")
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
  if(TARGET gaitos_mujoco_renderer)
    ament_add_gtest(test_renderer test/test_renderer.cpp)
    target_link_libraries(test_renderer gaitos_mujoco_renderer)
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(gaitos_core gaitos_system mujoco rclcpp_lifecycle sensor_msgs tf2_msgs pluginlib)
ament_package()
