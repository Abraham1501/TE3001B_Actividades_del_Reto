#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        self.Ts = 1/100

        self.sp = 0.0
        self.y  = 0.0
        self.u  = 0.0

        self.error = 0.0
        self.error_past = 0.0
        self.integral = 0.0

        self.Kp = 3.90
        self.Ki = 0.125
        self.Kd = 0.00075

        self.d_filter_tau = 0.05
        self.d_filt = 0.0       

        self.setPoint_sub = self.create_subscription(
            Float32, 'set_point', self.setPoint_callback, 10
        )
        self.get_logger().info("Controller node started. Waiting for set_point...")

        self.inputU_pub = self.create_publisher(Float32, 'motor_input_u', 10)
        self.get_logger().info("Controller node started. Waiting for motor_input_u...")

        self.speedY_sub = self.create_subscription(
            Float32, 'motor_speed_y', self.motorSpeedY_callback, 10
        )
        self.get_logger().info("Controller node started. Waiting for motor_speed_y...")

        self.control_timer = self.create_timer(self.Ts, self.control_loop)

    def setPoint_callback(self, msg):
        self.sp = msg.data

    def motorSpeedY_callback(self, msg):
        self.y = msg.data

    def control_loop(self):
        self.error = self.sp - self.y

        self.integral += self.error * self.Ts

        d_raw = (self.error - self.error_past) / self.Ts

        alpha = self.d_filter_tau / (self.d_filter_tau + self.Ts)
        self.d_filt = alpha * self.d_filt + (1.0 - alpha) * d_raw

        self.u = (self.Kp * self.error) + (self.Ki * self.integral) + (self.Kd * self.d_filt)

        self.error_past = self.error

        msgU = Float32()
        msgU.data = self.u
        self.inputU_pub.publish(msgU)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()