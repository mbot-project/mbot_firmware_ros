import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

# Robot parameters
LINEAR_SPEED = 0.2  # m/s
SIDE_LENGTH = 1.0   # meters
TURN_SPEED = 0.5    # rad/s (angular z)
TURN_ANGLE = 1.5708 # radians (90 deg)

# Calculate durations
DRIVE_TIME = SIDE_LENGTH / LINEAR_SPEED
TURN_TIME = TURN_ANGLE / TURN_SPEED

class SquareDriver(Node):
    def __init__(self):
        super().__init__('drive_square')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Starting square drive: 2 rounds, 1m per side')

    def drive_straight(self, duration):
        twist = Twist()
        twist.linear.x = LINEAR_SPEED
        twist.angular.z = 0.0
        self.get_logger().info(f'Driving straight for {duration:.2f} s')
        self._publish_for_duration(twist, duration)

    def turn_90_deg(self, duration):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = TURN_SPEED
        self.get_logger().info(f'Turning 90 deg for {duration:.2f} s')
        self._publish_for_duration(twist, duration)

    def _publish_for_duration(self, twist, duration, rate_hz=10):
        rate = 1.0 / rate_hz
        start = time.time()
        while time.time() - start < duration:
            self.publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0)  # allow ROS2 to process events
            time.sleep(rate)
        self.stop()

    def stop(self):
        twist = Twist()
        self.publisher.publish(twist)
        time.sleep(0.2)

    def run(self):
        for i in range(8):
            self.drive_straight(DRIVE_TIME)
            self.turn_90_deg(TURN_TIME)
        self.stop()
        self.get_logger().info('Finished driving square.')

def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, stopping robot.')
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
