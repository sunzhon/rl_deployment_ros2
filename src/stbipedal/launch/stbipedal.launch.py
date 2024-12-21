from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot="stbipedal"
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
            package=robot,
            executable=robot+"_node",
            name=robot+"_node",
            output="screen",
            emulate_tty=True,
            namespace=robot+"_ns",
            parameters=[config],
            arguments=['--ros-args', '--log-level', log_level]
        )
    ])

