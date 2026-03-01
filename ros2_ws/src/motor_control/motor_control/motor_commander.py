#!/usr/bin/env python3

"""
Motor Command Publisher Node

This simple ROS 2 node demonstrates how to control the motor
by publishing PWM commands to the /cmd_pwm topic.

Usage:
    ros2 run motor_control motor_commander -- --pwm <value>
    
Example:
    ros2 run motor_control motor_commander -- --pwm 200  # Forward at 200/255 PWM
    ros2 run motor_control motor_commander -- --pwm -150  # Reverse at 150/255 PWM
    ros2 run motor_control motor_commander -- --pwm 0     # Stop motor
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
import sys
import argparse


class MotorCommandPublisher(Node):
    def __init__(self, pwm_command=0):
        super().__init__('motor_commander')
        
        # Create publisher to /cmd_pwm topic
        self.cmd_pwm_publisher_ = self.create_publisher(
            Int16,
            'cmd_pwm',
            10)
        
        self.pwm_command = pwm_command
        
        # Create a timer to publish periodically
        timer_period = 0.1  # 100ms
        self.timer = self.create_timer(timer_period, self.publish_command)
        
        self.get_logger().info(f'Motor Commander Node Started')
        self.get_logger().info(f'Publishing command: {self.pwm_command} to /cmd_pwm')
    
    def publish_command(self):
        msg = Int16()
        msg.data = self.pwm_command
        self.cmd_pwm_publisher_.publish(msg)
        self.get_logger().debug(f'Published PWM command: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Publish motor PWM commands')
    parser.add_argument('--pwm', type=int, default=0, 
                        help='PWM command value (-255 to +255)')
    parsed_args = parser.parse_args(args=sys.argv[1:])
    
    # Validate PWM range
    pwm = parsed_args.pwm
    if pwm < -255 or pwm > 255:
        print(f"Error: PWM must be between -255 and +255, got {pwm}")
        sys.exit(1)
    
    # Create and run node
    motor_commander = MotorCommandPublisher(pwm)
    
    try:
        rclpy.spin(motor_commander)
    except KeyboardInterrupt:
        motor_commander.get_logger().info('Shutting down Motor Commander...')
    finally:
        motor_commander.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
