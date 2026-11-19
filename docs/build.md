# Build

[← gaitos](../README.md#documentation)

ROS 2 rolling (in `/opt/ros/rolling`; Gazebo and OSQP come from its vendor packages, installed
by rosdep). Other dependencies: MuJoCo 3 from the system (`libmujoco-dev`, no rosdep key) and
EGL for the `snapshot` tool. unitree_sdk2 is fetched by CMake (FetchContent) when
`gaitos_unitree` is configured, pinned to a commit (`UNITREE_SDK2_TAG`). The RMW used and
tested is CycloneDDS (`rmw_cyclonedds_cpp`).

```bash
cd ~/ws/legged_ws                                        # gaitos in src/gaitos
source /opt/ros/rolling/setup.bash
sudo apt install libmujoco-dev libegl-dev ros-rolling-rmw-cyclonedds-cpp
rosdep install --from-paths src --ignore-src -y          # ROS dependencies

colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --base-paths src && colcon test-result --all
source install/setup.bash
```

Offline, or with a local copy of the SDK, add
`-DFETCHCONTENT_SOURCE_DIR_UNITREE_SDK2=<path to unitree_sdk2>`. The SDK's CycloneDDS 0.10
(`libddsc.so.0`, `libddscxx.so.0`) is installed in `lib/gaitos_unitree/unitree_sdk2` and found
through the RPATH, so it never shadows the CycloneDDS 11 of ROS.

The messages of unitree_ros2 are fetched the same way (`UNITREE_ROS2_TAG`,
`-DFETCHCONTENT_SOURCE_DIR_UNITREE_ROS2=<path>`).

The meshes and MuJoCo assets of the Unitree robots (BSD-3 by Unitree, not versioned with
gaitos) are fetched by `gaitos_description`: archives of pinned commits of unitree_ros and
unitree_mujoco (`UNITREE_ROS_TAG`, `UNITREE_MUJOCO_TAG`; about 1.8 GB unpacked in its build
folder, of which ~100 MB are installed). Local copies:
`-DFETCHCONTENT_SOURCE_DIR_UNITREE_ROS=<path> -DFETCHCONTENT_SOURCE_DIR_UNITREE_MUJOCO=<path>`;
without them (no MuJoCo, no meshes in RViz): `-DGAITOS_FETCH_ASSETS=OFF`.

The NAO meshes (looks only, RViz and Gazebo) are not downloaded by default: their license is
Creative Commons BY-NC-ND 4.0, not that of gaitos, and building with
`-DGAITOS_NAO_MESHES=ON` means accepting it (see [Robots](robots.md)).

```bash
colcon build --base-paths src --packages-select gaitos_description --cmake-args -DGAITOS_NAO_MESHES=ON
```

## Continuous integration

[`.github/workflows/rolling.yml`](../.github/workflows/rolling.yml) does the same in the
`ros:rolling-ros-base` image on every push to `rolling`, every pull request and weekly:
rosdep, build, `colcon test`, and a smoke run of the Go2 in MuJoCo and in Gazebo (headless):
[`.github/scripts/smoke.py`](../.github/scripts/smoke.py) stands it up, walks with
`cmd_vel` and checks that it stands, walks and moves forward. Locally:

```bash
src/gaitos/.github/scripts/smoke_launch.sh gaitos_mujoco go2
src/gaitos/.github/scripts/smoke_launch.sh gaitos_gazebo go2 --min-distance 0.2 -- headless:=true
```
