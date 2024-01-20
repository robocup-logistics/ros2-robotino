  
import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
import launch_ros.descriptions

package_name = "robotino_description"

def generate_launch_description():
    package_share = get_package_share_directory(package_name)
    default_model_path = os.path.join(package_share, "urdf/robots/robotino3_description.urdf")
 
    robotino_driver_node = Node(
            package='rto_node',
            executable='rto_node',
            name='robotino_node',
            parameters=[{
                "hostname" : "172.26.108.81:12080"
            }]
        )
        
    robotino_odom_node = Node(
            package='rto_node',
            executable='rto_odometry_node',
            name='robotino_odometry_node',
            parameters=[{
                "hostname" : "172.26.108.81:12080"
            }]
        )
            
    # Initialize robot state publisher 
    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": Command(["xacro ", LaunchConfiguration("model")]),
                     "publish_frequency": 20.0}],
    )

    # Initialize joint state broadcaster
    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    #     name="joint_state_publisher",
    #     condition=launch.conditions.UnlessCondition(LaunchConfiguration("gui")),
    # )
    

    return LaunchDescription([
        
        # Declare launch arguments
        launch.actions.DeclareLaunchArgument(
            name="model", default_value=default_model_path, description="Absolute path to robot urdf file"
        ),
        
        # Launch nodes
        #joint_state_publisher_node,
        robot_state_publisher_node,
        robotino_driver_node,
        robotino_odom_node
    ])