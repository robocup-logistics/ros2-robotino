#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import socket
import struct
import sys
from rclpy.action import ActionClient
from gigatino_msgs.action import Move
from geometry_msgs.msg import Vector3
from gigatino_msgs.msg import Feedback

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

        self.move_client = ActionClient(self, Move,'gigatino/move')
        self.move_pub = self.create_publisher(Vector3, 'gigatino/move_absolute', 10)

        self.feedback_sub = self.create_subscription(
            Feedback,
            'gigatino/feedback',
            self.feedback_callback,
            10
        )

        self.current_x = 0.0
        self.current_yaw = 0.0
        self.current_z = 0.0
        self.prev_buttons = [0]*12

        self.step_x = 5.0     # mm
        self.step_z = 5.0     # mm
        self.step_yaw = 2.0   # deg

        # Initialize parameters
        self.declare_parameter('forward_axis_scalling', 1.0)
        self.declare_parameter('angular_axis_scalling', 1.0)

    def __del__(self):
        self.client_socket.close()

    def feedback_callback(self, msg):
        self.current_x = msg.stepper_positions[0]
        self.current_yaw = msg.stepper_positions[1]
        self.current_z = msg.stepper_positions[2]

    def send_absolute_move(self, x, yaw, z):

        msg = Vector3()
        msg.x = x
        msg.y = yaw
        msg.z = z

        self.move_pub.publish(msg)

        self.get_logger().info(
            f"Move absolute: x={x:.1f} yaw={yaw:.1f} z={z:.1f}"
        )

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

        if not self.release_img and data.buttons[4] == 0 and data.buttons[5] == 0 :
          self.release_img = True

        if data.buttons[4] == 1 and data.buttons[5] == 1:
          self.saved_img = True
        else:
          self.saved_img = False
        # Button press detection
        if data.buttons[2] == 1:   # X button
            if data.button[13] == 1:     # D-pad up
                self.send_absolute_move(self.current_x + self.step_x,
                                        self.current_yaw,
                                        self.current_z)

            elif data.buttons[14] == 1:  # D-pad down
                self.send_absolute_move(self.current_x - self.step_x,
                                        self.current_yaw,
                                        self.current_z)
        if data.buttons[1] == 1:   # X button
            if data.button[13] == 1:     # D-pad up
                self.send_absolute_move(self.current_x,
                                        self.current_yaw,
                                        self.current_z + self.step_z)

            elif data.buttons[14] == 1:  # D-pad down
                self.send_absolute_move(self.current_x,
                                        self.current_yaw,
                                        self.current_z - self.step_z)
        if data.buttons[3] == 1:   # X button
            if data.button[12] == 1:     # D-pad right
                self.send_absolute_move(self.current_x,
                                        self.current_yaw + self.step_yaw,
                                        self.current_z)

            elif data.buttons[11] == 1:  # D-pad down
                self.send_absolute_move(self.current_x,
                                        self.current_yaw - self.step_yaw,
                                        self.current_z)
        # if data.buttons[1] == 1 and self.prev_buttons[1] == 0:
        #     self.send_move_goal(0.05, 0.05, 0.1)
        #     self.prev_buttons = data.buttons
        # if data.buttons[2] == 1 and self.prev_buttons[2] == 0:
        #     self.send_move_goal(0.05, 0.05, 0.05)
        #     self.prev_buttons = data.buttons
        # if data.buttons[3] == 1 and self.prev_buttons[3] == 0:
        #     self.send_move_goal(0.05, 0.05, 0.01)
        #     self.prev_buttons = data.buttons
        if self.saved_img and self.release_img:
            message_type = 15
            message = struct.pack('!BQ', message_type,0)
            self.client_socket.sendall(message)
            self.get_logger().info('Img saved.')
            self.release_img = False
            self.saved_img = False
        self.publisher.publish(p_msg)

def send_move_goal(self, x, y, z):

    goal_msg = Move.Goal()

    goal_msg.target_frame = "robotinobase3/end_effector_home"
    goal_msg.x = x
    goal_msg.y = y
    goal_msg.z = z
    goal_msg.relative = False
    goal_msg.use_gripper = False
    goal_msg.gripper_state = False

    self.move_client.wait_for_server()

    self.get_logger().info(
        f"Sending goal: x={x}, y={y}, z={z}"
    )

    self.move_client.send_goal_async(goal_msg)

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
