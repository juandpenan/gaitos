# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)

cmake_minimum_required(VERSION 3.20)
project(gaitos_gazebo)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(gaitos_system REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)

# <plugin filename="gaitos_gz_system" name="gaitos::gazebo::GaitosGzSystem">
add_library(gaitos_gz_system SHARED src/GaitosGzSystem.cpp src/GazeboHardware.cpp)
target_include_directories(gaitos_gz_system PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(gaitos_gz_system PRIVATE
  gaitos_system::gaitos_system gz-sim::gz-sim gz-plugin::register)

install(TARGETS gaitos_gz_system LIBRARY DESTINATION lib)
install(DIRECTORY worlds launch params DESTINATION share/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")
ament_package()
