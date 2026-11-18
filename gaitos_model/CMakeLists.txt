# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)

cmake_minimum_required(VERSION 3.20)
project(gaitos_model)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(gaitos_common REQUIRED)
find_package(TinyXML2 REQUIRED)

add_library(gaitos_model SHARED
  src/Kinematics.cpp
  src/UrdfParser.cpp
  src/SrdfParser.cpp
  src/RobotModel.cpp
)
target_include_directories(gaitos_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(gaitos_model
  PUBLIC gaitos_common::gaitos_common
  PRIVATE tinyxml2::tinyxml2
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS gaitos_model EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_model test/test_model.cpp TIMEOUT 600)
  target_link_libraries(test_model gaitos_model )
  target_compile_definitions(test_model PRIVATE GAITOS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/..")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(gaitos_common)
ament_package()
