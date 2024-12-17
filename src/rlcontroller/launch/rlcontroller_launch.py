import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os.path as osp
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_dir = get_package_share_directory(
        'rlcontroller')
    policy_dir = osp.join(osp.dirname(osp.dirname(osp.dirname(osp.dirname(osp.dirname(package_share_dir))))),"legged_policies")
    policy_path = osp.join(policy_dir,"ambotw1","tag_b_1")
    print("policy path:", policy_path)

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rlcontroller', 
            node_executable='rlcontroller_node', 
            output='screen',
            node_namespace="/ambotw1_ns",
            node_name="rlcontroller_node",
            arguments=["--logdir", policy_path,
                '--log-level', 'DEBUG'
                ]
            )
    ])


