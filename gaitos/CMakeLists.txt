# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)

cmake_minimum_required(VERSION 3.20)
project(gaitos)

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})
ament_package()
