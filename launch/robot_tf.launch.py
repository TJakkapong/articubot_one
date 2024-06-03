import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def generate_launch_description():


    robot_tf_pos = Node(
                        package="robot_driver",
                        executable="robot_tf_pos",
    )    
    robot_tf_wheel = Node(
                        package="robot_driver",
                        executable="robot_tf_wheel",
    )

 
    # Launch!
    return LaunchDescription([
    robot_tf_pos,
    robot_tf_wheel,
        
    ])
