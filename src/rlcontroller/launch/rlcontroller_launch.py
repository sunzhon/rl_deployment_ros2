import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rlcontroller', 
            node_executable='rlcontroller_node', 
            output='screen',
            node_namespace="ambotw1_ns",
            node_name="rlcontroller_node",
            )
    ])

