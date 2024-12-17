from launch import LaunchDescription
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


    return LaunchDescription([
        Node(
            package="ambotw1",
            node_executable="ambotw1_node",
            node_name="ambotw1_node",
            output="screen",
            emulate_tty=True,
            node_namespace="ambotw1_ns",
            parameters=[
                config
            ]
        )
    ])
