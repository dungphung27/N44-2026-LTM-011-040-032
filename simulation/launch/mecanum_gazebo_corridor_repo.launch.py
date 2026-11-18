#!/usr/bin/env python3
"""Repo-local Gazebo corridor launcher: run after source sim_env.sh."""
import re
import sys
import tempfile
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SIM = Path(__file__).resolve().parents[1]
ROBOT = SIM / 'ros2_ws/src/mecanum_robot_viz/urdf'
GAZEBO_URDF = ROBOT / 'mecanum_robot_gazebo_real_visual.urdf'
RVIZ_URDF = ROBOT / 'mecanum_robot_rviz_real_visual.urdf'
WORLD = SIM / 'maps/my_map_corridor.world'
RELAY = SIM / 'scripts/mecanum_safe_cmd_relay.py'

def read_urdf(path):
    contents = path.read_text(encoding='utf-8-sig')
    return re.sub(r'^\s*<\?xml\s+[^?]*\?>\s*', '', contents, count=1)

def generate_launch_description():
    for path in (GAZEBO_URDF, RVIZ_URDF, WORLD):
        if not path.is_file():
            raise FileNotFoundError(path)
    with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
            prefix='ltmang_spawn_', suffix='.urdf', delete=False) as f:
        f.write(read_urdf(GAZEBO_URDF))
        spawn_file = f.name
    gazebo_launch = Path(get_package_share_directory('gazebo_ros')) / 'launch/gazebo.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=str(WORLD)),
        DeclareLaunchArgument('x', default_value='5.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.25'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('use_relay', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_launch)),
            launch_arguments={'world': LaunchConfiguration('world')}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen', parameters=[{
                 'robot_description': read_urdf(RVIZ_URDF), 'use_sim_time': True,
                 'publish_frequency': 40.0,
             }]),
        Node(package='gazebo_ros', executable='spawn_entity.py',
             name='spawn_mecanum_corridor', output='screen',
             arguments=['-file', spawn_file, '-entity', 'mecanum_robot',
                        '-x', LaunchConfiguration('x'),
                        '-y', LaunchConfiguration('y'),
                        '-z', LaunchConfiguration('z'),
                        '-Y', LaunchConfiguration('yaw')]),
        ExecuteProcess(cmd=[sys.executable, str(RELAY)],
                       name='mecanum_safe_cmd_relay', output='screen',
                       condition=IfCondition(LaunchConfiguration('use_relay'))),
    ])
