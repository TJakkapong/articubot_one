from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node
def generate_launch_description():
    # get path to rosbridge_server launch file
    launch_file_dir = os.path.join(get_package_share_directory('rosbridge_server'), 'launch')
    launch_file_path = os.path.join(launch_file_dir, 'rosbridge_websocket_launch.xml')
	
    # create an ExecuteProcess action to launch the rosbridge_server
    rosbridge_server_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', launch_file_path],
        output='screen'
    )
    react_server_cmd = Node(
                        package="website",
                        executable="web_start",
    )    
    

    # create a LaunchDescription that includes the ExecuteProcess action
    ld = LaunchDescription([
    	react_server_cmd,
        rosbridge_server_cmd,
   
    ])

    return ld

