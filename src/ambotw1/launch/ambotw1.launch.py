from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('ambotw1'),
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
