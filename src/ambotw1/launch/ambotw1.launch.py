from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ambotw1",
            node_executable="ambotw1_node",
            node_name="ambotw1_node",
            output="screen",
            emulate_tty=True,
            node_namespace="ambotw1_ns",
            parameters=[
                {"motor_device": "/dev/M1080"},
                {"imu_device": "/dev/YIS106"},
                {"motor_num": 12},
            ]
        )
    ])
