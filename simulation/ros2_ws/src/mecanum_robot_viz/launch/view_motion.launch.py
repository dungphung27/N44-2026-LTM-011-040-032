from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = Path(
        get_package_share_directory("mecanum_robot_viz")
    )

    urdf = (
        pkg / "urdf/mecanum_robot.urdf"
    ).read_text()

    return LaunchDescription([

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {
                    "robot_description": urdf,
                    "publish_frequency": 30.0,
                }
            ],
            output="screen"
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            output="screen"
        )

    ])
