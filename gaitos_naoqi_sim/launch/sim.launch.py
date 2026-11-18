# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)
"""A simulated NAO behind the ROS 2 interface of naoqi_driver.

    ros2 launch gaitos_naoqi_sim sim.launch.py [simulator:=mujoco|gazebo] [headless:=true] [rviz:=true]

gaitos runs the simulator in the namespace `gaitos` (/gaitos/mode, /gaitos/status, the mode
services) and, through the NaoqiDriverEmulator extension, publishes /joint_states, /odom,
/imu/torso and TF odom -> base_link -> base_footprint, and takes /cmd_vel, /joint_angles,
/move_base_simple/goal and /speech, as naoqi_driver does. A robot_state_publisher at the root
namespace (as the one inside naoqi_driver) publishes the TF of the links.

The robot starts in crouch (rest): stand it up with /gaitos/mode/stand_init (NAOqi wakeUp).
Use a ROS_DOMAIN_ID of its own: these are the topic names of naoqi_driver.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    simulator = LaunchConfiguration('simulator').perform(context)
    if simulator not in ('mujoco', 'gazebo'):
        raise RuntimeError(f"simulator must be mujoco or gazebo, not '{simulator}'")
    emulator = os.path.join(get_package_share_directory('gaitos_naoqi_sim'), 'params',
                            'nao_naoqi_sim.yaml')
    extra = [p.strip() for p in LaunchConfiguration('params').perform(context).split(',') if p.strip()]
    args = {'robot': 'nao', 'namespace': LaunchConfiguration('namespace').perform(context),
            'params': ','.join([emulator] + extra), 'robot_state_publisher': 'false'}
    if simulator == 'gazebo':
        args['headless'] = LaunchConfiguration('headless').perform(context)
    with open(os.path.join(get_package_share_directory('gaitos_description'), 'robots',
                           'nao_description', 'urdf', 'nao.urdf')) as f:
        urdf = f.read()
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory(f'gaitos_{simulator}'), 'launch', 'sim.launch.py')),
            launch_arguments=args.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='log',
             parameters=[{'robot_description': urdf, 'use_sim_time': True}]),
    ]
    if LaunchConfiguration('rviz').perform(context) == 'true':
        actions.append(Node(
            package='rviz2', executable='rviz2', output='log',
            arguments=['-d', os.path.join(get_package_share_directory('gaitos_description'),
                                          'rviz', 'gaitos.rviz')],
            parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulator', default_value='mujoco', description='mujoco or gazebo'),
        DeclareLaunchArgument('namespace', default_value='gaitos',
                              description='of the gaitos node (its modes and status)'),
        DeclareLaunchArgument('params', default_value='', description='extra parameter files'),
        DeclareLaunchArgument('headless', default_value='false', description='Gazebo only'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
