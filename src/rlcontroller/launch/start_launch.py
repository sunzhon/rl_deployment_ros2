from launch import LaunchDescription
import os
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
 
def generate_launch_description():
    """find launch files"""
    robot="ambotw1"
    robot_pkg_share = FindPackageShare(package=robot).find(robot)
    robot_launch_file_path = os.path.join(robot_pkg_share, 'launch', robot+'.launch.py')    
    
    joyremap="ros2joyremap"
    joyremap_pkg_share = FindPackageShare(package=joyremap).find(joyremap)
    joyremap_launch_file_path = os.path.join(joyremap_pkg_share,  joyremap+'.launch.py')    


    controlller="rlcontroller"
    controller_pkg_share = FindPackageShare(package=controlller).find(controlller)
    controller_launch_file_path = os.path.join(controller_pkg_share, controlller+'_launch.py')    
 
    # include launch files
    robot_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch_file_path),
        # 可以在这里传递参数，例如：launch_arguments={'arg_name': 'arg_value'}.items(),
    )
 
    joyremap_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joyremap_launch_file_path),
    )


    controller_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controller_launch_file_path),
    )
    
    ld = LaunchDescription()
    ld.add_action(robot_launch_file)
    ld.add_action(joyremap_launch_file)
    #ld.add_action(controller_launch_file)
 
    return ld
