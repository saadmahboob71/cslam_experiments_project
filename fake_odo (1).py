import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from tf_transformations import quaternion_from_euler
import math

class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom_publisher')
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.1, self.publish_odom)  # 10 Hz

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 1.0  # 1 m/s
        self.angular_velocity = 0.1  # 0.1 rad/s

    def publish_odom(self):
        current_time = self.get_clock().now().to_msg()

        # Update pose
        dt = 0.1  # Time step (assuming 10 Hz)
        self.x += self.linear_velocity * math.cos(self.theta) * dt
        self.y += self.linear_velocity * math.sin(self.theta) * dt
        self.theta += self.angular_velocity * dt

        # Create the odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # Set the position
        odom.pose.pose = Pose(
            position=Point(x=self.x, y=self.y, z=0.0),
            orientation=Quaternion(*quaternion_from_euler(0, 0, self.theta))
        )

        # Set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(
            linear=Vector3(x=self.linear_velocity, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=self.angular_velocity)
        )

        # Publish the message
        self.odom_publisher.publish(odom)
        self.get_logger().info(f'Publishing: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = FakeOdomPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


