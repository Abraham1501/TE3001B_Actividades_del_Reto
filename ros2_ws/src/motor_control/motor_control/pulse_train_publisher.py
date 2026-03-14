#!/usr/bin/env python3
"""
Pulse-train RPM reference publisher.

Publishes a square-wave (pulse train) to /cmd_vel_rpm, alternating
between a HIGH and LOW RPM value at a configurable period and duty
cycle.

Parameters
----------
high_rpm      : RPM during the HIGH phase  (default  60.0)
low_rpm       : RPM during the LOW  phase  (default   0.0)
period        : Total pulse period [s]      (default   4.0)
duty_cycle    : Fraction of period that is HIGH, in (0, 1]  (default 0.5)
publish_rate  : Timer publish rate [Hz]     (default 100)

Example
-------
ros2 run motor_control pulse_train_publisher \
    --ros-args -p high_rpm:=80.0 -p low_rpm:=0.0 \
               -p period:=5.0   -p duty_cycle:=0.5
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class PulseTrainPublisher(Node):
    def __init__(self):
        super().__init__('pulse_train_publisher')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('high_rpm',    70.0)   # RPM during ON phase
        self.declare_parameter('low_rpm',     -70.0)   # RPM during OFF phase
        self.declare_parameter('period',       15.0)   # Full cycle [s]
        self.declare_parameter('duty_cycle',   0.5)   # 0.0 < duty <= 1.0
        self.declare_parameter('publish_rate', 100)   # Hz

        self.high_rpm     = self.get_parameter('high_rpm').value
        self.low_rpm      = self.get_parameter('low_rpm').value
        self.period       = self.get_parameter('period').value
        self.duty_cycle   = max(0.01, min(1.0, self.get_parameter('duty_cycle').value))
        self.publish_rate = self.get_parameter('publish_rate').value

        # ── Publisher & timer ─────────────────────────────────────────
        self.publisher = self.create_publisher(Float32, '/cmd_vel_rpm', 10)
        self.timer     = self.create_timer(1.0 / self.publish_rate, self._timer_callback)

        self.start_time = time.time()

        self.get_logger().info(
            f'Pulse-train publisher started:\n'
            f'  High RPM   : {self.high_rpm}\n'
            f'  Low RPM    : {self.low_rpm}\n'
            f'  Period     : {self.period} s\n'
            f'  Duty cycle : {self.duty_cycle * 100:.1f} %\n'
            f'  ON time    : {self.period * self.duty_cycle:.3f} s\n'
            f'  OFF time   : {self.period * (1.0 - self.duty_cycle):.3f} s\n'
            f'  Publish rate: {self.publish_rate} Hz'
        )

    def _timer_callback(self):
        elapsed = time.time() - self.start_time

        # Position within the current period [0, period)
        phase = elapsed % self.period

        # HIGH during [0, duty_cycle * period), LOW otherwise
        rpm = self.high_rpm if phase < self.duty_cycle * self.period else self.low_rpm

        msg = Float32()
        msg.data = float(rpm)
        self.publisher.publish(msg)

        self.get_logger().debug(f'phase={phase:.3f}s  rpm={rpm:.1f}')


def main(args=None):
    rclpy.init(args=args)
    node = PulseTrainPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
