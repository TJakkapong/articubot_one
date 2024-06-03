import os
 
from ament_index_python.packages import get_package_share_directory
 
 
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
 
from launch_ros.actions import Node
 
 
 
def generate_launch_description():
 
 
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
 
    package_name='articubot_one' #<--- CHANGE ME
    package_name2='nav2_bringup' #<--- CHANGE ME
 
    launch_realbot = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','launch_robot.launch.py'
                )])
    )    
     
    localization = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','localization_launch.py'
                )])
    ) 
    
    navigation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name2),'launch','navigation_launch.py'
                )])
    ) 


    # Launch them all!
    return LaunchDescription([
        launch_realbot,
        localization,navigation
        
     
    ])
