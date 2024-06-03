from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joy_params = os.path.join(get_package_share_directory('articubot_one'),'config','joystick.yaml')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('twist_mux'),'launch','twist_mux_launch.py'
                )]),
    )
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params],
    )
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        remappings=[('/cmd_vel', '/joy_vel')],
    )





    return LaunchDescription([
        joy_node,teleop_node,rsp
    ])
