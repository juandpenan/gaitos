# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)

cmake_minimum_required(VERSION 3.20)
project(gaitos_unitree)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(ament_cmake REQUIRED)
find_package(gaitos_core REQUIRED)

# unitree_sdk2: fetched at configure time (prebuilt static library + its CycloneDDS 0.10).
# Offline or with a local copy: -DFETCHCONTENT_SOURCE_DIR_UNITREE_SDK2=<path to unitree_sdk2>
include(FetchContent)
set(UNITREE_SDK2_TAG aaf69cb56c9d5ed63d8b284008ec3a8ab999bec1 CACHE STRING "unitree_sdk2 commit")
FetchContent_Declare(unitree_sdk2
  GIT_REPOSITORY https://github.com/unitreerobotics/unitree_sdk2.git
  GIT_TAG ${UNITREE_SDK2_TAG}
  EXCLUDE_FROM_ALL)   # no install rules of the SDK (headers, examples) in this package
set(BUILD_EXAMPLES OFF CACHE BOOL "unitree_sdk2 examples" FORCE)
FetchContent_MakeAvailable(unitree_sdk2)

# The SDK's CycloneDDS 0.10 (libddsc.so.0, libddscxx.so.0) goes to a private folder, so it never
# shadows the CycloneDDS 11 of ROS; the binaries find it through their RPATH.
set(UNITREE_DDS_DIR lib/${PROJECT_NAME}/unitree_sdk2)
file(GLOB UNITREE_DDS_LIBS "${unitree_sdk2_SOURCE_DIR}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}/libddsc*.so*")
install(FILES ${UNITREE_DDS_LIBS} DESTINATION ${UNITREE_DDS_DIR})

add_library(gaitos_unitree SHARED src/UnitreeHardware.cpp)
target_include_directories(gaitos_unitree PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(gaitos_unitree PUBLIC gaitos_core::gaitos_core PRIVATE unitree_sdk2)
set_target_properties(gaitos_unitree PROPERTIES INSTALL_RPATH "$ORIGIN/${PROJECT_NAME}/unitree_sdk2")
pluginlib_export_plugin_description_file(gaitos_core plugins.xml)

# Headless Go2 simulator speaking unitree_sdk2 DDS like the real robot (needs MuJoCo)
find_package(mujoco QUIET)
if(mujoco_FOUND)
  add_executable(go2_dds_sim tools/go2_dds_sim.cpp)
  target_link_libraries(go2_dds_sim PRIVATE unitree_sdk2 mujoco::mujoco)
  set_target_properties(go2_dds_sim PROPERTIES INSTALL_RPATH "$ORIGIN/unitree_sdk2")
  install(TARGETS go2_dds_sim RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})
install(TARGETS gaitos_unitree EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(gaitos_core)
ament_package()
