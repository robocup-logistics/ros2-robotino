#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class Robotino3Teleop(Node):

    def __init__(self):
        super().__init__('robotino_joyteleop', namespace='')
        
        # create subscription to joy topic
        self.subscription = self.create_subscription(Joy, 'joy', self.TeleopCallback, 10)
        
        # create publisher to cmd_vel topic
        self.publisher= self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize parameters
        self.declare_parameter('forward_axis_scalling', 1.0)
        self.declare_parameter('angular_axis_scalling', 1.0)
        
    # callback function to publish data over cmd_vel topic based on joy_pad inputs
    def TeleopCallback(self, data):
        f_scale = self.get_parameter('forward_axis_scalling').value
        z_scale = self.get_parameter('angular_axis_scalling').value
        p_msg = Twist()
    
        p_msg.linear.x = data.axes[1]*f_scale
        p_msg.linear.y = data.axes[0]*f_scale
        p_msg.linear.z = 0.0
        
        p_msg.angular.x = 0.0
        p_msg.angular.y = 0.0
        p_msg.angular.z = data.axes[3]*z_scale

        self.publisher.publish(p_msg)

def main():
    rclpy.init()
    teleop_node = Robotino3Teleop()
    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
