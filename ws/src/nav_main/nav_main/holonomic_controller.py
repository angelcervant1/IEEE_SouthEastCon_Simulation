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

        self.subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 15)
        self.publisher = self.create_publisher(Float64MultiArray, '/r1/commands', 15)
        self.gz_publisher = self.create_publisher(Twist, '/r1/cmd_vel', 15)
        self.tf_broadcaster = TransformBroadcaster(self)

        # base constants
        self.wheel_radius = 0.05  # 5 cm radius
        self.base_length = 0.5    # distance between wheels (x-axis)
        self.base_width = 0.5     # distance between wheels (y-axis)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        self.tf_timeout = 0.1  # 100 ms timeout
        self.last_cmd_time = self.get_clock().now()  
        self.timer = self.create_timer(0.1, self.update_pose_and_tf)  # 10 hz rate

    def cmd_vel_callback(self, msg):
        
        # Velocities are remapped because of tf not being passed correctly by robot_state_publisher
        self.linear_x = msg.linear.y
        self.linear_y = -msg.linear.x
        self.angular_z = msg.angular.z

        self.last_cmd_time = self.get_clock().now()

        # wheel kinematics matrix
        wheel_matrix = np.array([
            [1, -1, -(self.base_length + self.base_width)],  # Front Left
            [1,  1,  (self.base_length + self.base_width)],  # Front Right
            [1,  1, -(self.base_length + self.base_width)],  # Rear Left
            [1, -1,  (self.base_length + self.base_width)]   # Rear Right
        ]) / self.wheel_radius

        velocity_vector = np.array([self.linear_x, self.linear_y, self.angular_z])
        wheel_velocities = wheel_matrix @ velocity_vector

        command = Float64MultiArray()
        command.data = wheel_velocities.tolist()

        try:
            self.publisher.publish(command)
            self.gz_publisher.publish(msg)
            # self.get_logger().info(f"Published wheel velocities: {command.data}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish command: {e}")

    def update_pose_and_tf(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-10  # each 100 ms
        self.last_time = current_time

        time_since_last_cmd = (current_time - self.last_cmd_time).nanoseconds * 1e-9
        
        # reset tf after long delay of teleop cmd_vel
        if time_since_last_cmd > self.tf_timeout:
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.angular_z = 0.0

        self.x += (self.linear_x * np.cos(self.theta) - self.linear_y * np.sin(self.theta)) * dt
        self.y += (self.linear_x * np.sin(self.theta) + self.linear_y * np.cos(self.theta)) * dt
        self.theta += self.angular_z * dt
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi

        self.broadcast_transform()

    def broadcast_transform(self):
        current_time = self.get_clock().now()

        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = np.sin(self.theta / 2.0)
        t.transform.rotation.w = np.cos(self.theta / 2.0)

        pose = [t.transform.translation.x, t.transform.translation.y, self.theta]
        # self.get_logger().info(f"Robot Current Pose (x, y, theta): {pose}")
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = HolonomicController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
        node.destroy_node()
        # rclpy.shutdown()


if __name__ == '__main__':
    main()
