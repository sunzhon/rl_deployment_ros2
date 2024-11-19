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
            parameters=[
                {"motor_device": "/dev/ttyUSB0"},
                {"imu_device": "/dev/ttyUSB1"},
                {"motor_num": 12},
            ]
        )
    ])
