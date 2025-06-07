import rclpy
from rclpy.node import Node
from mbot_interfaces.msg import Encoders
import csv
import os
from datetime import datetime

class EncoderLogger(Node):
    def __init__(self):
        super().__init__('encoder_logger')
        self.subscription = self.create_subscription(
            Encoders,
            '/encoders',
            self.listener_callback,
            10)
        # Create a timestamped CSV file in the current directory
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_filename = f'encoder_log_{timestamp}.csv'
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # Write header
        self.csv_writer.writerow([
            'ros_time',
            'ticks_L', 'ticks_R', 'ticks_UNUSED',
            'delta_ticks_L', 'delta_ticks_R', 'delta_ticks_UNUSED',
            'delta_time_usec'
        ])
        self.get_logger().info(f'Logging to {self.csv_filename}')

    def listener_callback(self, msg: Encoders):
        now = self.get_clock().now().to_msg()
        row = [
            f'{now.sec}.{now.nanosec:09d}',
            msg.ticks[0], msg.ticks[1], msg.ticks[2],
            msg.delta_ticks[0], msg.delta_ticks[1], msg.delta_ticks[2],
            msg.delta_time
        ]
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EncoderLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down logger...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
