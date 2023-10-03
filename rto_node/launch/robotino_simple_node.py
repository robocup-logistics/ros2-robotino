from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rto_node',
            executable='rto_node',
            name='robotino_node'
        ),
        Node(
            package='rto_node',
            executable='rto_odometry_node',
            name='robotino_odometry_node'
        ),
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     parameters=[{
        #         "publish_frequency", "20"
        #     }]
        # )
    ])