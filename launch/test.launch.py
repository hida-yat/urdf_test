#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
from launch.substitutions import Command, LaunchConfiguration
import launch
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    pkg_share = launch_ros.substitutions.FindPackageShare(package='urdf_test').find('urdf_test')
    default_model_path = os.path.join(pkg_share, 'urdf/test.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/config.rviz')
    with open(default_model_path, 'r') as infp:
        robot_desc = infp.read()

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 
        'use_sim_time': True}] # add other parameters here if required
    )

    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # Run the node
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        node_robot_state_publisher,
        joint_state_publisher_gui_node, 
        rviz_node
    ])


