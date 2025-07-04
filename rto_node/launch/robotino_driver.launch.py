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
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
import launch
import xacro
from launch.substitutions import LaunchConfiguration, Command


def launch_nodes_withconfig(context, *args, **kwargs):

    # Declare launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_jsb = LaunchConfiguration('launch_jsb')
    robot_description = LaunchConfiguration('robot_description')
    hostname = LaunchConfiguration('hostname')
    launch_teleopnode = LaunchConfiguration('launch_teleopnode')
    launch_joynode = LaunchConfiguration('launch_joynode')
    joy_deadzone = LaunchConfiguration('joy_deadzone')
    launch_rsp_freq = LaunchConfiguration('rsp_freq')
    launch_odom_tf = LaunchConfiguration('launch_odom_tf')
    bumper_timeout = LaunchConfiguration('bumper_timeout')
    motor_timeout = LaunchConfiguration('motor_timeout')
    # Process the Xacro file
    xacro_description = xacro.process_file(robot_description.perform(context), mappings={}, in_order=True, base_path=os.path.dirname(robot_description.perform(context))).toxml()


    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval

    tf_prefix = launch_configuration['namespace']+'/'

    # launch robotinobase controllers with individual namespaces
    launch_nodes = GroupAction(
        actions=[

        # Launch robotino driver node
        Node(
            package='rto_node',
            executable='rto_node',
            name='robotino_node',
            namespace=namespace,
            parameters=[{
                'hostname' : hostname,
                'tf_prefix' : tf_prefix,
                'bumper_timeout' : bumper_timeout,
                'motor_timeout' : motor_timeout
            }],
        ),

        Node(
            package='rto_node',
            executable='rto_odometry_node',
            name='robotino_odometry_node',
            namespace=namespace,
            parameters=[{
                'hostname' : hostname,
                'tf_prefix' : launch_configuration['namespace']+'/', 
                'publish_odom_tf': launch_odom_tf
            }]
        ),

        # Initialize robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            parameters=[{'robot_description': xacro_description,
                        'use_sim_time': use_sim_time,
                        'publish_frequency': launch_rsp_freq,
                        'frame_prefix': tf_prefix}],
        ),

        # Initialize joint state broadcaster
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            namespace=namespace,
            condition = IfCondition(launch_jsb),
        ),

         # Joy node to enable joystick teleop
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="log",
            namespace=namespace,
            parameters=[{"deadzone": joy_deadzone}],
            condition= IfCondition(launch_joynode)
        ),

        # Joy teleop node to enable joystick teleop
        Node(package="rto_node",
            executable="robotino_teleop.py",
            name ="robotino_teleop",
            output ="log",
            namespace=namespace,
            condition= IfCondition(launch_teleopnode)
        ),

    ])

    return[launch_nodes]

def generate_launch_description():
    pkg_share_description = get_package_share_directory('rto_description')

    # Declare launch configuration variables
    declare_namespace_argument = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true')

    declare_launch_jsb_argument = DeclareLaunchArgument(
        'launch_jsb',
        default_value='false',
        description= 'Whether to start the joint state broadcaster or not')

    declare_robot_description_config_argument = DeclareLaunchArgument(
        'robot_description',default_value=os.path.join(pkg_share_description, "urdf/robots/robotino_description.urdf"),
        description='Full path to robot description')

    declare_hostname_argument = DeclareLaunchArgument(
        'hostname', default_value='172.26.1.1:12080',
        description='ip address of robotino')

    declare_launch_rsp_freq_argument = DeclareLaunchArgument(
        'rsp_freq', default_value='20.0',
        description='publish frequency for robot state publisher node')

    declare_launch_joynode_argument = DeclareLaunchArgument(
        'launch_joynode',
        default_value='true',
        description= 'Whether to start joynode based on launch environment')

    declare_joy_deadzone_argument = DeclareLaunchArgument(
        'joy_deadzone',
        default_value='0.2',
        description= 'deadzone for joy node to avoid movement when idle due to joystick jitter')

    declare_launch_teleopnode_argument = DeclareLaunchArgument(
        'launch_teleopnode',
        default_value='true',
        description= 'Weather to start teleop node')
    
    declare_bumper_timeout_argument = DeclareLaunchArgument(
        'bumper_timeout',
        default_value='2.0',
        description= 'Time to stop robot when bumper is hit')
    
    declare_motor_timeout_argument = DeclareLaunchArgument(
        'motor_timeout',
        default_value='0.0',
        description= 'Time to stop robot when motor is not responding')

    declare_launch_odom_tf_argument = DeclareLaunchArgument(
        'launch_odom_tf',
        default_value='false',
        description= 'Whether to broadcast the tf for odom frame')


    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_launch_jsb_argument)
    ld.add_action(declare_robot_description_config_argument)
    ld.add_action(declare_hostname_argument)
    ld.add_action(declare_launch_joynode_argument)
    ld.add_action(declare_joy_deadzone_argument)
    ld.add_action(declare_launch_teleopnode_argument)
    ld.add_action(declare_launch_rsp_freq_argument)
    ld.add_action(declare_bumper_timeout_argument)
    ld.add_action(declare_launch_odom_tf_argument)
    ld.add_action(declare_motor_timeout_argument)


    # Add the actions to launch webots, controllers and rviz
    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))

    return ld
