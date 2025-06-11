"""
This script is used to drive the robot in a square pattern.

To run the script, use the following command:

python3 drive_square.py

"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math
from tf_transformations import euler_from_quaternion

# Robot parameters
LINEAR_SPEED = 0.2  # m/s
SIDE_LENGTH = 1.0   # meters
ANGULAR_SPEED = 0.5 # rad/s
TURN_ANGLE = math.pi / 2  # 90 degrees in radians

class SquareDriver(Node):
    def __init__(self):
        super().__init__('drive_square')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_publisher = self.create_publisher(Path, '/path', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'  # or 'map' if you use map frame
        self.state = 'IDLE'  # IDLE, DRIVE, TURN, DONE
        self.sides_completed = 0
        self.start_pos = None
        self.start_yaw = None
        self.current_pos = None
        self.current_yaw = None
        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info('Starting closed-loop square drive: 1m per side')

    def odom_callback(self, msg):
        self.current_pos = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

        # Add pose to path
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.poses.append(pose_stamped)
        self.path_publisher.publish(self.path_msg)

    def loop(self):
        if self.current_pos is None or self.current_yaw is None:
            return  # Wait for odom
        if self.state == 'IDLE':
            self.start_pos = (self.current_pos.x, self.current_pos.y)
            self.state = 'DRIVE'
            self.get_logger().info(f'Starting side {self.sides_completed + 1}')
        elif self.state == 'DRIVE':
            dx = self.current_pos.x - self.start_pos[0]
            dy = self.current_pos.y - self.start_pos[1]
            dist = math.hypot(dx, dy)
            if dist < SIDE_LENGTH:
                twist = Twist()
                twist.linear.x = LINEAR_SPEED
                self.publisher.publish(twist)
            else:
                self.stop()
                self.start_yaw = self.current_yaw
                self.state = 'TURN'
                self.get_logger().info('Starting turn')
        elif self.state == 'TURN':
            # Compute angle turned (handle wraparound)
            angle_turned = self.normalize_angle(self.current_yaw - self.start_yaw)
            if abs(angle_turned) < TURN_ANGLE:
                twist = Twist()
                twist.angular.z = ANGULAR_SPEED
                self.publisher.publish(twist)
            else:
                self.stop()
                self.sides_completed += 1
                if self.sides_completed >= 4:
                    self.state = 'DONE'
                    self.get_logger().info('Finished driving square.')
                else:
                    self.state = 'IDLE'
        elif self.state == 'DONE':
            self.stop()
            self.destroy_node()
            rclpy.shutdown()

    def stop(self):
        twist = Twist()
        self.publisher.publish(twist)

    @staticmethod
    def normalize_angle(angle):
        # Normalize angle to [-pi, pi]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, stopping robot.')
        node.stop()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
