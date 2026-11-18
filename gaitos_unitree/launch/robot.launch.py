# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)
"""A real Unitree robot through unitree_sdk2.

    ros2 launch gaitos_unitree robot.launch.py robot:=go2 [readonly:=true] [dds_sim:=true]
                                               [network_interface:=eth0] [joy:=true]
                                               [params:=extra.yaml,...]

readonly: the robot is read but never commanded (check joints, IMU, feet, remote first).
dds_sim: against a simulator speaking the same DDS topics (unitree_mujoco or go2_dds_sim):
interface lo, domain 1.
"""
import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def gaitos_launch():
    """The gaitos launch module (gaitos_nodes, description)"""
    path = os.path.join(get_package_share_directory("gaitos"), "launch", "gaitos.launch.py")
    spec = importlib.util.spec_from_file_location("gaitos_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_setup(context):
    robot = LaunchConfiguration('robot').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context) or robot
    share = os.path.join(get_package_share_directory('gaitos_unitree'), 'params')
    params = [os.path.join(share, f'{robot}_unitree.yaml')]
    if LaunchConfiguration('dds_sim').perform(context) == 'true':
        params.append(os.path.join(share, f'{robot}_unitree_mujoco.yaml'))
    if LaunchConfiguration('readonly').perform(context) == 'true':
        params.append(os.path.join(share, f'{robot}_unitree_readonly.yaml'))
    params += [p.strip() for p in LaunchConfiguration('params').perform(context).split(',') if p.strip()]
    overrides = {}
    interface = LaunchConfiguration('network_interface').perform(context)
    if interface:
        overrides['hardware.network_interface'] = interface
    nodes = gaitos_launch().gaitos_nodes(robot, namespace, params, overrides=overrides,
                                         rviz=LaunchConfiguration('rviz').perform(context) == 'true')
    if (LaunchConfiguration('dds_sim').perform(context) != 'true'
            and os.environ.get('RMW_IMPLEMENTATION', '') == 'rmw_cyclonedds_cpp'
            and os.environ.get('ROS_DOMAIN_ID', '0') in ('', '0')):
        nodes.insert(0, LogInfo(msg=(
            'WARNING: ROS 2 shares DDS domain 0 with the robot. A ROS subscriber on a robot topic '
            '(e.g. ros2 topic echo /lowcmd with unitree_ros2) crashes the CycloneDDS 0.10 of '
            'unitree_sdk2, and with it this node. Prefer another ROS_DOMAIN_ID (e.g. 10).')))
    if LaunchConfiguration('joy').perform(context) == 'true':
        nodes.append(Node(package='joy', executable='joy_node', namespace=namespace, output='log'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='go2', description='go2, r1'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('readonly', default_value='false'),
        DeclareLaunchArgument('dds_sim', default_value='false'),
        DeclareLaunchArgument('network_interface', default_value=''),
        DeclareLaunchArgument('params', default_value='', description='extra parameter files'),
        DeclareLaunchArgument('joy', default_value='false', description='also a joy_node (teleop.joy.enabled)'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
