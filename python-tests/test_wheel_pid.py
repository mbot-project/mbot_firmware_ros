"""

This script is used to test the wheel velocity PID controller.

To run the script, use the following command:

python3 test_wheel_pid.py

"""
import rclpy
from rclpy.node import Node
from mbot_interfaces.msg import MotorVelocity
import time

class WheelPIDTester(Node):
    def __init__(self):
        super().__init__('wheel_pid_tester')
        
        # Publisher for wheel commands
        self.wheel_cmd_pub = self.create_publisher(MotorVelocity, '/motor_vel_cmd', 10)
        
        # Subscriber for feedback
        self.wheel_vel_sub = self.create_subscription(MotorVelocity, '/motor_vel', 
                                                     self.wheel_vel_callback, 10)
        
        self.current_wheel_vel = [0.0, 0.0, 0.0]
        self.timer = self.create_timer(0.1, self.control_loop)
        self.start_time = time.time()
        self.test_complete = False
        
        self.get_logger().info('Testing wheel velocity PID controller...')

    def wheel_vel_callback(self, msg):
        self.current_wheel_vel = msg.velocity.copy()

    def send_wheel_command(self, left_vel, right_vel):
        msg = MotorVelocity()
        msg.velocity[0] = left_vel   # Left wheel
        msg.velocity[1] = right_vel  # Right wheel
        msg.velocity[2] = 0.0        # Unused
        self.wheel_cmd_pub.publish(msg)

    def control_loop(self):
        if self.test_complete:
            return
            
        elapsed = time.time() - self.start_time
        
        # Simple test sequence
        if elapsed < 5.0:
            # Forward
            cmd_left, cmd_right = 2, -2
            self.get_logger().info(f'Forward: cmd=[{cmd_left:.1f}, {cmd_right:.1f}], actual=[{self.current_wheel_vel[0]:.2f}, {self.current_wheel_vel[1]:.2f}]')
        elif elapsed < 10.0:
            # Turn right
            cmd_left, cmd_right = 2, 2
            self.get_logger().info(f'Turn right: cmd=[{cmd_left:.1f}, {cmd_right:.1f}], actual=[{self.current_wheel_vel[0]:.2f}, {self.current_wheel_vel[1]:.2f}]')
        elif elapsed < 15.0:
            # Backward
            cmd_left, cmd_right = -2, 2
            self.get_logger().info(f'Backward: cmd=[{cmd_left:.1f}, {cmd_right:.1f}], actual=[{self.current_wheel_vel[0]:.2f}, {self.current_wheel_vel[1]:.2f}]')
        else:
            # Stop
            cmd_left, cmd_right = 0.0, 0.0
            self.get_logger().info('Test complete - stopping')
            self.test_complete = True
            
        self.send_wheel_command(cmd_left, cmd_right)

def main(args=None):
    rclpy.init(args=args)
    tester = WheelPIDTester()
    
    try:
        while rclpy.ok() and not tester.test_complete:
            rclpy.spin_once(tester, timeout_sec=0.1)
    except KeyboardInterrupt:
        tester.get_logger().info('Interrupted')
    finally:
        # Stop the robot
        tester.send_wheel_command(0.0, 0.0)
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 