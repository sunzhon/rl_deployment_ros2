from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot="ambotw1"
    config = os.path.join(
        FindPackageShare(package=robot).find(robot),
        'config',
        'params.yaml'
        )

    log_level="warn"
    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value=["debug"],
            description="Logging level",
            ),
        Node(
            package="ambotw1",
            node_executable="ambotw1_node",
            node_name="ambotw1_node",
            output="screen",
            emulate_tty=True,
            node_namespace="ambotw1_ns",
            parameters=[config],
            arguments=['--ros-args', '--log-level', log_level]
        )
    ])

