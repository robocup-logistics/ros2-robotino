import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    grips_launch_dir = get_package_share_directory('rto_description')
    model_path = os.path.join(grips_launch_dir, 'urdf', 'robots', 'rto-1.urdf.xacro')
    
    return LaunchDescription([
        Node(
            package='rto_node',
            executable='rto_node',
            name='robotino_node',
            parameters=[{
                "hostname" : "192.168.5.90:12080"
            }]
        ),
        Node(
            package='rto_node',
            executable='rto_odometry_node',
            name='robotino_odometry_node',
            parameters=[{
                "hostname" : "192.168.5.90:12080"
            }]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {"publish_frequency" : 20.0,
                'robot_description': ParameterValue(Command(['xacro ', model_path]), value_type=str)}
            ]
        )
    ])