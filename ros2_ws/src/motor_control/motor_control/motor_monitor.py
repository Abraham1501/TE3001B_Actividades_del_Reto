#!/usr/bin/env python3

"""
Motor Feedback Monitor Node

Subscribes to motor feedback topics and prints real-time status.
Use rqt_plot for live graphs:

    rqt_plot /motor/rpm/data /cmd_pwm/data

Topics monitored:
  - /motor/rpm     (std_msgs/Float32) : Actual RPM
  - /motor/encoder (std_msgs/Int32)   : Encoder pulse count
  - /motor/state   (std_msgs/Int16)   : 0=stopped, 1=running
  - /cmd_pwm       (std_msgs/Int16)   : PWM reference command

Usage:
    ros2 run motor_control motor_monitor
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16, Int32


class MotorMonitor(Node):
    def __init__(self):
        super().__init__('motor_monitor')

        self.create_subscription(Float32, 'motor/rpm',     self.rpm_callback,     10)
        self.create_subscription(Int32,   'motor/encoder', self.encoder_callback, 10)
        self.create_subscription(Int16,   'motor/state',   self.state_callback,   10)
        self.create_subscription(Int16,   'cmd_pwm',       self.cmd_callback,     10)

        self.last_rpm     = 0.0
        self.last_encoder = 0
        self.last_state   = 0
        self.last_cmd     = 0

        self.get_logger().info('Motor Monitor started.')
        self.get_logger().info('Tip: run  rqt_plot /motor/rpm/data /cmd_pwm/data  for live graphs.')

    def rpm_callback(self, msg):
        self.last_rpm = msg.data
        self._print_status()

    def encoder_callback(self, msg):
        self.last_encoder = msg.data

    def state_callback(self, msg):
        self.last_state = msg.data

    def cmd_callback(self, msg):
        self.last_cmd = msg.data

    def _print_status(self):
        state_str = 'RUNNING' if self.last_state == 1 else 'STOPPED'
        print(
            f'[Motor] State: {state_str:8} | '
            f'RPM: {self.last_rpm:7.2f} | '
            f'Encoder: {self.last_encoder:7} | '
            f'CMD: {self.last_cmd:5}'
        )


def main(args=None):
    rclpy.init(args=args)
    motor_monitor = MotorMonitor()

    try:
        rclpy.spin(motor_monitor)
    except KeyboardInterrupt:
        motor_monitor.get_logger().info('Shutting down Motor Monitor...')
    finally:
        motor_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
