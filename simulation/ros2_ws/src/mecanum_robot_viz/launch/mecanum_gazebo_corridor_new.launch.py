#!/usr/bin/env python3
"""Gazebo Classic corridor world + the corrected Mecanum simulation robot.

ROS 2 Humble; run only on the simulation PC (ROS_DOMAIN_ID=31).
Gazebo uses the fixed-joint physical URDF; RViz/TF uses the matching
continuous-wheel URDF. Nothing is launched on the real robot.
"""

from pathlib import Path
import re
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


HOME = Path.home()
URDF_DIR = HOME / 'mecanum_viz_ws/src/mecanum_robot_viz/urdf'
GAZEBO_URDF = URDF_DIR / 'mecanum_robot_gazebo_real_visual.urdf'
RVIZ_URDF = URDF_DIR / 'mecanum_robot_rviz_real_visual.urdf'
DEFAULT_WORLD = HOME / 'maps/my_map_corridor.world'
RELAY = HOME / 'mecanum_safe_cmd_relay.py'


def without_xml_declaration(path: Path) -> str:
    # gazebo_ros/spawn_entity.py on Humble rejects encoding declarations in
    # Unicode input. Remove only that declaration from the copy.
    contents = path.read_text(encoding='utf-8-sig')
    return re.sub(r'^\s*<\?xml\s+[^?]*\?>\s*', '', contents, count=1)


def generate_launch_description():
    for path in (GAZEBO_URDF, RVIZ_URDF):
        if not path.is_file():
            raise FileNotFoundError(f'Missing simulation URDF: {path}')

    # Spawn Gazebo from its own plugin-enabled URDF, NOT robot_description.
    # The latter belongs to RViz2 and intentionally contains no Gazebo plugins.
    with tempfile.NamedTemporaryFile(
        mode='w', encoding='utf-8', prefix='mecanum_corridor_spawn_',
        suffix='.urdf', delete=False,
    ) as temporary:
        temporary.write(without_xml_declaration(GAZEBO_URDF))
        gazebo_spawn_file = temporary.name

    gazebo_launch = (
        Path(get_package_share_directory('gazebo_ros'))
        / 'launch/gazebo.launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=str(DEFAULT_WORLD)),
        DeclareLaunchArgument('x', default_value='5.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.25'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        # Enable only when no separate Safe Command Relay is already running.
        DeclareLaunchArgument('use_relay', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_launch)),
            launch_arguments={'world': LaunchConfiguration('world')}.items(),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': without_xml_declaration(RVIZ_URDF),
                'use_sim_time': True,
                'publish_frequency': 40.0,
            }],
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_mecanum_corridor',
            output='screen',
            arguments=[
                '-file', gazebo_spawn_file,
                '-entity', 'mecanum_robot',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-Y', LaunchConfiguration('yaw'),
            ],
        ),

        ExecuteProcess(
            cmd=[sys.executable, str(RELAY)],
            name='mecanum_safe_cmd_relay',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_relay')),
        ),
    ])
