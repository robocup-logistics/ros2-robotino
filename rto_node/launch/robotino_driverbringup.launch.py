# Author: Saurabh Borse(saurabh.borse@alumni.fh-aachen.de)

#  MIT License
#  Copyright (c) 2023 Saurabh Borse
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:

#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.

#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.

#!/usr/bin/env python

import os
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch import LaunchDescription
from launch.substitutions.path_join_substitution import PathJoinSubstitution


def launch_nodes_withconfig(context, *args, **kwargs):

    # Declare launch configuration variables
    namespace = LaunchConfiguration('namespace')
    hostname = LaunchConfiguration('hostname')
    launch_teleopnode = LaunchConfiguration('launch_teleopnode')
    launch_joynode = LaunchConfiguration('launch_joynode')
    launch_odom_tf = LaunchConfiguration('launch_odom_tf')

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval

    tf_prefix = launch_configuration['namespace']+'/'

    # launch robotinobase controllers with individual namespaces
    launch_nodes = GroupAction(
        actions=[

        # Launch Integrate laserscan launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rto_node'),'launch','robotino_driver.launch.py'
                ])
            ]),
            launch_arguments={
                'namespace': namespace,
                'hostname': hostname,
                'launch_teleopnode': launch_teleopnode,
                'launch_joynode': launch_joynode,
                'launch_odom_tf': launch_odom_tf,
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rto_node'),'launch','robotino_tfbroadcaster.launch.py'
                ])
            ]),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': 'false',
            }.items()
        )
    ])

    return[launch_nodes]

def generate_launch_description():

    # Declare launch configuration variables
    declare_namespace_argument = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace')

    declare_hostname_argument = DeclareLaunchArgument(
        'hostname', default_value='172.26.1.1:12080',
        description='ip addres of robotino')

    declare_launch_joynode_argument = DeclareLaunchArgument(
        'launch_joynode',
        default_value='true',
        description= 'Wheather to start joynode based on launch environment')

    declare_launch_teleopnode_argument = DeclareLaunchArgument(
        'launch_teleopnode',
        default_value='true',
        description= 'Wheather to start teleop node not based on launch environment')

    declare_launch_odom_tf_argument = DeclareLaunchArgument(
        'launch_odom_tf',
        default_value='true',
        description= 'Wheather to broadcast transform based on launch environment')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_launch_joynode_argument)
    ld.add_action(declare_launch_teleopnode_argument)

    # Add the actions to launch webots, controllers and rviz
    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))

    return ld
