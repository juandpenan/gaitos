# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)

cmake_minimum_required(VERSION 3.20)
project(gaitos_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY robots launch rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "meshes" EXCLUDE PATTERN "assets" EXCLUDE)   # the assets come from FetchContent
install(PROGRAMS scripts/make_gazebo_urdf.py scripts/make_mjcf.py DESTINATION lib/${PROJECT_NAME})

# Meshes and MuJoCo assets of the Unitree robots (BSD-3-Clause, Unitree Robotics), not versioned
# with gaitos: downloaded at configure time (archives of pinned commits) and installed next to
# the URDF / MJCF of each robot. Offline or with local copies:
#   -DFETCHCONTENT_SOURCE_DIR_UNITREE_ROS=<unitree_ros> -DFETCHCONTENT_SOURCE_DIR_UNITREE_MUJOCO=<unitree_mujoco>
# Without them (no MuJoCo models, no meshes in RViz / Gazebo): -DGAITOS_FETCH_ASSETS=OFF
option(GAITOS_FETCH_ASSETS "Download the Unitree meshes and MuJoCo assets" ON)
if(GAITOS_FETCH_ASSETS)
  include(FetchContent)
  set(UNITREE_ROS_TAG 157ff950351e5f535e03c5d85a16b7d594b3dbcc CACHE STRING "unitree_ros commit")
  set(UNITREE_MUJOCO_TAG 1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d CACHE STRING "unitree_mujoco commit")
  FetchContent_Declare(unitree_ros
    URL https://github.com/unitreerobotics/unitree_ros/archive/${UNITREE_ROS_TAG}.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR no_cmake)   # only download
  FetchContent_Declare(unitree_mujoco
    URL https://github.com/unitreerobotics/unitree_mujoco/archive/${UNITREE_MUJOCO_TAG}.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR no_cmake)
  FetchContent_MakeAvailable(unitree_ros unitree_mujoco)

  set(UROS ${unitree_ros_SOURCE_DIR}/robots)
  set(UMJ ${unitree_mujoco_SOURCE_DIR}/unitree_robots)
  set(ROBOTS share/${PROJECT_NAME}/robots)
  # <from> <to>: folder contents
  set(ASSETS
    ${UROS}/go2_description/dae   ${ROBOTS}/go2_description/meshes
    ${UMJ}/go2/assets             ${ROBOTS}/go2_description/mjcf/assets
    ${UROS}/r1_description/meshes ${ROBOTS}/r1_description/meshes
    ${UMJ}/r1/meshes              ${ROBOTS}/r1_description/mjcf/meshes)
  while(ASSETS)
    list(POP_FRONT ASSETS from to)
    if(NOT IS_DIRECTORY ${from})
      message(FATAL_ERROR "Not found in the Unitree repositories: ${from}")
    endif()
    install(DIRECTORY ${from}/ DESTINATION ${to})
  endwhile()
  install(FILES ${unitree_ros_SOURCE_DIR}/LICENSE DESTINATION share/${PROJECT_NAME}/licenses
    RENAME unitree_ros.LICENSE)
  install(FILES ${unitree_mujoco_SOURCE_DIR}/LICENSE DESTINATION share/${PROJECT_NAME}/licenses
    RENAME unitree_mujoco.LICENSE)
endif()

# NAO meshes (SoftBank Robotics, nao_meshes installer), only for the looks: the simulation
# models of the NAO use primitive collisions. Their license is not the one of gaitos but
# Creative Commons BY-NC-ND 4.0 (non commercial, no derivatives): building with
# -DGAITOS_NAO_MESHES=ON means you accept it. Installed with that license next to them.
# Offline: -DGAITOS_NAO_MESHES_INSTALLER=<naomeshes-0.6.7-linux-x64-installer.run>
option(GAITOS_NAO_MESHES "Download the NAO meshes, accepting their CC BY-NC-ND 4.0 license" OFF)
if(GAITOS_NAO_MESHES)
  if(NOT CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "x86_64")
    message(FATAL_ERROR "The NAO meshes installer only exists for x86_64")
  endif()
  set(NAO_INSTALLER ${CMAKE_CURRENT_BINARY_DIR}/nao_meshes/installer.run)
  if(GAITOS_NAO_MESHES_INSTALLER)
    configure_file(${GAITOS_NAO_MESHES_INSTALLER} ${NAO_INSTALLER} COPYONLY)
  elseif(NOT EXISTS ${NAO_INSTALLER})
    file(DOWNLOAD
      https://github.com/ros-naoqi/nao_meshes_installer/raw/master/naomeshes-0.6.7-linux-x64-installer.run
      ${NAO_INSTALLER} INACTIVITY_TIMEOUT 60 EXPECTED_MD5 5f897a3328f217bae6c0ed2a934e524c)
  endif()
  set(NAO_MESHES ${CMAKE_CURRENT_BINARY_DIR}/nao_meshes/out)
  add_custom_command(OUTPUT ${NAO_MESHES}/meshes/V40
    COMMAND chmod +x ${NAO_INSTALLER}
    COMMAND ${NAO_INSTALLER} --mode unattended --prefix ${NAO_MESHES}
    COMMAND ${CMAKE_COMMAND} -E rm -rf ${NAO_MESHES}/uninstall ${NAO_MESHES}/rollback*
    DEPENDS ${NAO_INSTALLER}
    COMMENT "NAO meshes: unpacking (license CC BY-NC-ND 4.0 accepted with GAITOS_NAO_MESHES)")
  add_custom_target(nao_meshes ALL DEPENDS ${NAO_MESHES}/meshes/V40)
  install(DIRECTORY ${NAO_MESHES}/meshes ${NAO_MESHES}/texture
    DESTINATION share/${PROJECT_NAME}/robots/nao_description OPTIONAL)
  install(FILES robots/nao_description/MESHES_LICENSE.md
    DESTINATION share/${PROJECT_NAME}/robots/nao_description/meshes RENAME LICENSE.md)
endif()

# The URDFs refer to their meshes as package://<robot>_description/...: each robot folder is
# also a package of the ament index (a link share/<robot>_description -> robots/<robot>_description),
# so RViz (resource_retriever) resolves them like Gazebo does through GZ_SIM_RESOURCE_PATH.
# A package of that name already in the environment (e.g. r1_description of unitree_r1, with
# the same meshes) is left alone.
foreach(robot go2 r1 nao)
  set(pkg ${robot}_description)
  ament_index_has_resource(other packages ${pkg})
  if(other AND NOT other MATCHES "${PROJECT_NAME}")
    message(STATUS "${pkg}: the package of ${other} provides the meshes")
    continue()
  endif()
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/ament_index/${pkg} "")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ament_index/${pkg}
    DESTINATION share/ament_index/resource_index/packages)
  install(CODE "file(CREATE_LINK ${PROJECT_NAME}/robots/${pkg}
    \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/${pkg} SYMBOLIC)")
endforeach()

ament_package()
