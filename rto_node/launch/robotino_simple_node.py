import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterValue
import pathlib

def generate_launch_description():
    package_dir = get_package_share_directory('robotino_description')
    #model_path = os.path.join(package_dir, 'urdf', 'robots', 'rto-1.urdf.xacro')
    
    # Load robot description file
    def load_file(filename):
        return pathlib.Path(os.path.join(package_dir, 'urdf/robots', filename)).read_text()

    
    return LaunchDescription([
        Node(
            package='rto_node',
            executable='rto_node',
            name='robotino_node',
            parameters=[{
                "hostname" : "172.26.108.81:12080"
            }]
        ),
        Node(
            package='rto_node',
            executable='rto_odometry_node',
            name='robotino_odometry_node',
            parameters=[{
                "hostname" : "172.26.108.81:12080"
            }]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {"publish_frequency" : 20.0,
                'robot_description': load_file('robotino3_description.urdf')}
            ]
        )
    ])