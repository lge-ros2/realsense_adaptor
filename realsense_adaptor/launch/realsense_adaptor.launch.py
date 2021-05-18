#
# LGE Advanced Robotics Laboratory
# Copyright (c) 2020 LG Electronics Inc., LTD., Seoul, Korea
# All Rights are Reserved.
#
# SPDX-License-Identifier: MIT
#

import os
import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Get the launch directory
    _package_name = "realsense_adaptor"
    config_file_path = os.path.join(get_package_share_directory(_package_name), 'config', "realsense_adaptor.yaml")

    _namespace = LaunchConfiguration('robot_name')

    start_node_cmd = Node(
        package=_package_name,
        node_executable=_package_name,
        node_name=_package_name,
        node_namespace=_namespace,
        parameters=[config_file_path],
        output='screen')

    declare_launch_argument = DeclareLaunchArgument('robot_name', default_value='', description='')

    # Create the launch description and populate
    ld = launch.LaunchDescription()

    ld.add_action(declare_launch_argument)
    ld.add_action(start_node_cmd)

    return ld
