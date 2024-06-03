#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    channel_type_1 =  LaunchConfiguration('channel_type_1', default='serial')
    serial_port_1 = LaunchConfiguration('serial_port_1', default='/dev/ttyUSB0')
    serial_baudrate_1 = LaunchConfiguration('serial_baudrate_1', default='115200')
    frame_id_1 = LaunchConfiguration('frame_id_1', default='laser_frame')
    inverted_1 = LaunchConfiguration('inverted_1', default='false')
    angle_compensate_1 = LaunchConfiguration('angle_compensate_1', default='true')
    
    return LaunchDescription([
        
        DeclareLaunchArgument(
            'channel_type_1',
            default_value=channel_type_1,
            description='Specifying channel type of lidar 1'),
        
        DeclareLaunchArgument(
            'serial_port_1',
            default_value=serial_port_1,
            description='Specifying usb port to connected lidar 1'),

        DeclareLaunchArgument(
            'serial_baudrate_1',
            default_value=serial_baudrate_1,
            description='Specifying usb port baudrate to connected lidar 1'),
        
        DeclareLaunchArgument(
            'frame_id_1',
            default_value=frame_id_1,
            description='Specifying frame_id of lidar 1'),

        DeclareLaunchArgument(
            'inverted_1',
            default_value=inverted_1,
            description='Specifying whether or not to invert scan data for lidar 1'),

        DeclareLaunchArgument(
            'angle_compensate_1',
            default_value=angle_compensate_1,
            description='Specifying whether or not to enable angle_compensate of scan data for lidar 1'),
        
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node_1',
            parameters=[{'channel_type':channel_type_1,
                         'serial_port': serial_port_1,
                         'serial_baudrate': serial_baudrate_1,
                         'frame_id': frame_id_1,
                         'inverted': inverted_1,
                         'angle_compensate': angle_compensate_1}],
            output='screen'),

    ])
