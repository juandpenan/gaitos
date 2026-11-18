# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)
"""Display a robot in RViz: gaitos on fake hardware (no physics) in one of its SRDF poses.

    ros2 launch gaitos_description display.launch.py robot:=r1 pose:=squat
    ros2 service call /r1/mode/stand_up std_srvs/srv/Trigger   # the posture modes move it
"""
import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def gaitos_launch():
    """The gaitos launch module (gaitos_nodes, description)"""
    path = os.path.join(get_package_share_directory("gaitos"), "launch", "gaitos.launch.py")
    spec = importlib.util.spec_from_file_location("gaitos_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


DEFAULT_POSE = {'go2': 'stand', 'r1': 'stand', 'nao': 'stand_init'}


def launch_setup(context):
    robot = LaunchConfiguration('robot').perform(context)
    pose = LaunchConfiguration('pose').perform(context) or DEFAULT_POSE.get(robot, 'stand')
    fake = os.path.join(get_package_share_directory('gaitos'), 'params', 'fake.yaml')
    return gaitos_launch().gaitos_nodes(robot, robot, [fake], rviz=True,
                                        overrides={'hardware.initial_pose': pose})


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='go2', description='go2, r1, nao'),
        DeclareLaunchArgument('pose', default_value='', description='SRDF pose'),
        OpaqueFunction(function=launch_setup),
    ])
