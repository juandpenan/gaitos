#!/bin/bash
# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)
# A Gazebo clip of gaitos for the documentation: the robot world plus a fixed camera that saves
# its images (headless rendering, bridged to ROS and saved by save_images.py), the robot driven through ROS (drive.py), the frames cropped
# around the robot (make_gif.py --follow).
#
#   CAM="x y z roll pitch yaw" record_gazebo.sh <robot> <world> <out.gif> <step>...
#   e.g. CAM="1.2 -3 0.9 0 0.15 1.5708" record_gazebo.sh go2 empty docs/media/go2_gazebo_walk.gif \
#          wait:12 mode:stand_up wait:4 vel:0.3,0,0 wait:6 stop wait:2
# Needs the workspace sourced and a ROS_DOMAIN_ID of its own.
set -e
robot=$1; world=$2; out=$3; shift 3
cam=${CAM:-"1.2 -3.0 0.9 0 0.15 1.5708"}
here=$(cd "$(dirname "$0")" && pwd)
tmp=$(mktemp -d)
mkdir -p "$tmp/frames"
src="$(ros2 pkg prefix gaitos_gazebo)/share/gaitos_gazebo/worlds/${world}.sdf"
python3 - "$src" "$tmp/doc.sdf" "$tmp/frames" "$cam" <<'PY'
import sys
src, dst, frames, cam = sys.argv[1:]
s = open(src).read()
camera = f'''
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <model name="doc_camera">
      <static>true</static>
      <pose>{cam}</pose>
      <link name="link">
        <sensor name="camera" type="camera">
          <update_rate>10</update_rate>
          <always_on>true</always_on>
          <camera>
            <horizontal_fov>1.0</horizontal_fov>
            <image><width>1280</width><height>720</height></image>
            <clip><near>0.05</near><far>50</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
  </world>'''
open(dst, 'w').write(s.replace('  </world>', camera, 1))
PY
name=$(grep -o '<world name="[^"]*"' "$tmp/doc.sdf" | head -1 | cut -d'"' -f2)
topic="/world/$name/model/doc_camera/link/link/sensor/camera/image"
ros2 launch gaitos_gazebo sim.launch.py robot:="$robot" world:="$tmp/doc.sdf" headless:=true \
  > "$tmp/launch.log" 2>&1 &
launch=$!
ros2 run ros_gz_bridge parameter_bridge "$topic@sensor_msgs/msg/Image[gz.msgs.Image" \
  > "$tmp/bridge.log" 2>&1 &
bridge=$!
python3 "$here/save_images.py" "$topic" "$tmp/frames" > "$tmp/saver.log" 2>&1 &
saver=$!
python3 "$here/drive.py" "$robot" "$@"
kill -INT $saver $bridge $launch; wait $launch 2>/dev/null || true
frames=$(ls -v "$tmp"/frames/*.png)
# shellcheck disable=SC2086
python3 "$here/make_gif.py" "$out" $frames --fps 10 --follow ${GIF_OPTS:-}
rm -rf "$tmp"
