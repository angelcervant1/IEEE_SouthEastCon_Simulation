#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# holonomic_controller.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np

class HolonomicController(Node):
    def __init__(self):
        super().__init__('holonomic_controller')
        self.get_logger().info('HolonomicController node has started.')
        self.subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.publisher = self.create_publisher(Float64MultiArray, '/holonomic_velocity_controller/commands', 10)

        self.wheel_radius = 0.05  # 5cm radius
        self.base_length = 0.5  # Distance between wheels (x-axis)
        self.base_width = 0.5   # Distance between wheels (y-axis)

    def cmd_vel_callback(self, msg):
        if not self.publisher:
            self.get_logger().error("Publisher is not available.")
            return

        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        wheel_matrix = np.array([
            [1, -1, -(self.base_length + self.base_width)],  # Front Left
            [1,  1,  (self.base_length + self.base_width)],  # Front Right
            [1,  1, -(self.base_length + self.base_width)],  # Rear Left
            [1, -1,  (self.base_length + self.base_width)]   # Rear Right
        ]) / self.wheel_radius

        velocity_vector = np.array([linear_x, linear_y, angular_z])
        wheel_velocities = wheel_matrix @ velocity_vector

        command = Float64MultiArray()
        command.data = wheel_velocities.tolist()

        try:
            self.publisher.publish(command)
            self.get_logger().info(f"Published wheel velocities: {command.data}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish command: {e}")

    def destroy_node(self):
        if rclpy.ok():
            self.get_logger().info('HolonomicController node is shutting down.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HolonomicController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
