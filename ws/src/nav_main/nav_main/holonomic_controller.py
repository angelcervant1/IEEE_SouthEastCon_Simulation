#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# holonomic_controller.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class HolonomicController(Node):
    def __init__(self):
        super().__init__('holonomic_controller')
        self.get_logger().info('HolonomicController node has started.')
        self.subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.publisher = self.create_publisher(Float64MultiArray, '/r1/commands', 10)
        self.gz_publisher = self.create_publisher(Twist, '/r1/cmd_vel', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.wheel_radius = 0.05  # 5cm radius
        self.base_length = 0.5  # Distance between wheels (x-axis)
        self.base_width = 0.5   # Distance between wheels (y-axis)
        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
    def cmd_vel_callback(self, msg):
        # Extract velocities
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        # Update pose (assumes fixed time step; replace with actual dt if needed)
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        self.x += (linear_x * np.cos(self.theta) - linear_y * np.sin(self.theta)) * dt
        self.y += (linear_x * np.sin(self.theta) + linear_y * np.cos(self.theta)) * dt
        self.theta += angular_z * dt

        # Broadcast TF
        self.broadcast_transform()

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
            self.gz_publisher.publish(msg)
            self.get_logger().info(f"Published wheel velocities: {command.data}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish command: {e}")

    def destroy_node(self):
        if rclpy.ok():
            self.get_logger().info('HolonomicController node is shutting down.')
        super().destroy_node()
   
    def broadcast_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = np.sin(self.theta / 2.0)
        t.transform.rotation.w = np.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(t)
        
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
