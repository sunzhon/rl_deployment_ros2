from launch import LaunchDescription
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


    return LaunchDescription([
        Node(
            package=robot,
            node_executable=robot+"_node",
            node_name=robot+"_node",
            output="screen",
            emulate_tty=True,
            node_namespace=robot+"_ns",
            parameters=[
                config
            ]
        )
    ])
