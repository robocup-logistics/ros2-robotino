#!/usr/bin/env python3

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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import array
import math
import socket
import struct
import sys

class Robotino3Teleop(Node):

    def __init__(self):
        super().__init__('robotino_joyteleop', namespace='')
        
        # create subscription to joy topic
        self.subscription = self.create_subscription(Joy, 'joy', self.TeleopCallback, 10)
        
        # create publisher to cmd_vel topic
        self.publisher= self.create_publisher(Twist, 'cmd_vel', 10)

        self.saved_img = False
        self.release_img = True
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(("192.168.0.100", 6465))
        
        # Initialize parameters
        self.declare_parameter('forward_axis_scalling', 1.0)
        self.declare_parameter('angular_axis_scalling', 1.0)

    def __del__(self):
        self.client_socket.close()
        
    # callback function to publish data over cmd_vel topic based on joy_pad inputs
    def TeleopCallback(self, data):
        f_scale = self.get_parameter('forward_axis_scalling').value
        z_scale = self.get_parameter('forward_axis_scalling').value
        p_msg = Twist()
    
        p_msg.linear.x = data.axes[1]*f_scale
        p_msg.linear.y = data.axes[0]*f_scale
        p_msg.linear.z = 0.0
        
        p_msg.angular.x = 0.0
        p_msg.angular.y = 0.0
        p_msg.angular.z = data.axes[3]*z_scale

        if not self.release_img and data.buttons[4] == 0 and data.buttons[5] == 0 :
          self.release_img = True

        if data.buttons[4] == 1 and data.buttons[5] == 1:
          self.saved_img = True
        else:
          self.saved_img = False
        if self.saved_img and self.release_img:
            message_type = 15
            message = struct.pack('!BQ', message_type,0)
            self.client_socket.sendall(message)
            self.get_logger().info('Img saved.')
            self.release_img = False
            self.saved_img = False
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
