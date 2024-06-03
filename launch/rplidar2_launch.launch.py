#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    channel_type_2 =  LaunchConfiguration('channel_type_2', default='serial')
    serial_port_2 = LaunchConfiguration('serial_port_2', default='/dev/ttyUSB2')
    serial_baudrate_2 = LaunchConfiguration('serial_baudrate_2', default='115200')
    frame_id_2 = LaunchConfiguration('frame_id_2', default='laser_frame')
    inverted_2 = LaunchConfiguration('inverted_2', default='false')
    angle_compensate_2 = LaunchConfiguration('angle_compensate_2', default='true')
    
    return LaunchDescription([

        DeclareLaunchArgument(
            'channel_type_2',
            default_value=channel_type_2,
            description='Specifying channel type of lidar 2'),
        
        DeclareLaunchArgument(
            'serial_port_2',
            default_value=serial_port_2,
            description='Specifying usb port to connected lidar 2'),

        DeclareLaunchArgument(
            'serial_baudrate_2',
            default_value=serial_baudrate_2,
            description='Specifying usb port baudrate to connected lidar 2'),
        
        DeclareLaunchArgument(
            'frame_id_2',
            default_value=frame_id_2,
            description='Specifying frame_id of lidar 2'),

        DeclareLaunchArgument(
            'inverted_2',
            default_value=inverted_2,
            description='Specifying whether or not to invert scan data for lidar 2'),

        DeclareLaunchArgument(
            'angle_compensate_2',
            default_value=angle_compensate_2,
            description='Specifying whether or not to enable angle_compensate of scan data for lidar 2'),

      
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node_2',
            parameters=[{'channel_type':channel_type_2,
                         'serial_port': serial_port_2,
                         'serial_baudrate': serial_baudrate_2,
                         'frame_id': frame_id_2,
                         'inverted': inverted_2,
                         'angle_compensate': angle_compensate_2}],
            output='screen'),
    ])
