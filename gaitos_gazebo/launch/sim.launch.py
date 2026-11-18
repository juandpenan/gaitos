# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)
"""A robot in Gazebo (gz-sim): the gaitos node runs inside the simulator (GaitosGzSystem plugin,
lockstep), with ros_gz_sim, the /clock bridge, robot_state_publisher and optionally RViz.

    ros2 launch gaitos_gazebo sim.launch.py robot:=go2 [headless:=true] [rviz:=true]
    ros2 launch gaitos_gazebo sim.launch.py robot:=r1 world:=empty_fine
    ros2 service call /go2/mode/stand_up std_srvs/srv/Trigger

The simulated model is the robot URDF, or its physics variant urdf/<robot>_gazebo.urdf when it
exists (made by gaitos_description make_gazebo_urdf.py), plus the plugin.
"""
import importlib.util
import os
import re
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def gaitos_launch():
    """The gaitos launch module (gaitos_nodes, description)"""
    path = os.path.join(get_package_share_directory("gaitos"), "launch", "gaitos.launch.py")
    spec = importlib.util.spec_from_file_location("gaitos_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


SPAWN_Z = {'go2': 0.25, 'r1': 0.72, 'nao': 0.34}
DEFAULT_WORLD = {'go2': 'empty', 'r1': 'empty_fine', 'nao': 'empty_fine'}


def launch_setup(context):
    robot = LaunchConfiguration('robot').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context) or robot
    world = LaunchConfiguration('world').perform(context) or DEFAULT_WORLD.get(robot, 'empty')
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    z = LaunchConfiguration('z').perform(context) or str(SPAWN_Z.get(robot, 0.3))
    gl = gaitos_launch()
    base, urdf, srdf = gl.description(robot)
    share = get_package_share_directory('gaitos_gazebo')

    # Parameters of the node inside Gazebo: the same chain as gaitos.launch.py, as files
    tmp = tempfile.mkdtemp(prefix='gaitos_gazebo_')
    desc_file = os.path.join(tmp, 'description.yaml')
    with open(desc_file, 'w') as f:
        yaml.safe_dump({'/**': {'ros__parameters': {
            'robot_description': urdf, 'robot_description_semantic': srdf}}}, f)
    params = [os.path.join(get_package_share_directory('gaitos'), 'params', 'gaitos.yaml'),
              os.path.join(base, 'config', f'{robot}.yaml'),
              os.path.join(share, 'params', f'{robot}_gazebo.yaml')]
    params += [p.strip() for p in LaunchConfiguration('params').perform(context).split(',') if p.strip()]
    params.append(desc_file)

    # Simulated model: physics variant of the URDF (if any) plus the plugin
    variant = os.path.join(base, 'urdf', f'{robot}_gazebo.urdf')
    with open(variant if os.path.exists(variant) else os.path.join(base, 'urdf', f'{robot}.urdf')) as f:
        model = f.read()
    plugin = ('  <gazebo>\n    <plugin filename="gaitos_gz_system" name="gaitos::gazebo::GaitosGzSystem">\n'
              f'      <ros>\n        <namespace>{namespace}</namespace>\n'
              + ''.join(f'        <parameters>{p}</parameters>\n' for p in params)
              + '      </ros>\n    </plugin>\n  </gazebo>\n</robot>')
    model = model[:model.rfind('</robot>')] + plugin
    model_file = os.path.join(tmp, f'{robot}.urdf')
    with open(model_file, 'w') as f:
        f.write(model)

    world_file = world if world.endswith('.sdf') else os.path.join(share, 'worlds', f'{world}.sdf')
    with open(world_file) as f:
        world_name = re.search(r'<world\s+name="([^"]+)"', f.read()).group(1)
    gz_args = f'-r {world_file}' + (' -s --headless-rendering' if headless else '')
    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(
            get_package_share_directory('gaitos_description'), 'robots') + ':' +
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', os.path.join(
            get_package_prefix('gaitos_gazebo'), 'lib') + ':' +
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', world_name, '-file', model_file, '-name', namespace,
                        '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                        '-z', z, '-Y', LaunchConfiguration('yaw')]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='log',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
    ]
    if LaunchConfiguration('robot_state_publisher').perform(context) == 'true':
        actions.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=namespace, output='log',
            parameters=[{'robot_description': urdf, 'use_sim_time': True}]))
    if LaunchConfiguration('rviz').perform(context) == 'true':
        actions.append(Node(
            package='rviz2', executable='rviz2', output='log', namespace=namespace,
            arguments=['-d', os.path.join(get_package_share_directory('gaitos_description'),
                                          'rviz', 'gaitos.rviz')],
            parameters=[{'use_sim_time': True}],
            remappings=[('/robot_description', f'/{namespace}/robot_description')]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='go2', description='go2, r1'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('world', default_value='', description='empty, empty_fine or an SDF path'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('params', default_value='', description='extra parameter files'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value=''),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('robot_state_publisher', default_value='true',
                              description='false when another stack publishes the robot state'),
        OpaqueFunction(function=launch_setup),
    ])
