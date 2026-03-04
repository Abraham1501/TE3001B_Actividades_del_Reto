#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import time


class SineWavePublisher(Node):
    def __init__(self):
        super().__init__('sine_wave_publisher')
        
        # Declare and get parameters
        self.declare_parameter('amplitude', 30.0)       # Peak RPM
        self.declare_parameter('frequency', 0.1)        # Hz
        self.declare_parameter('offset', 30.0)           # DC offset
        self.declare_parameter('publish_rate', 100)     # Hz (publish frequency)
        
        self.amplitude = self.get_parameter('amplitude').value
        self.frequency = self.get_parameter('frequency').value
        self.offset = self.get_parameter('offset').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Create publisher
        self.publisher = self.create_publisher(Float32, '/cmd_vel_rpm', 10)
        
        # Create timer for publication
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        # Track start time
        self.start_time = time.time()
        
        self.get_logger().info(
            f'Sine wave publisher started:\n'
            f'  Amplitude: {self.amplitude} RPM\n'
            f'  Frequency: {self.frequency} Hz\n'
            f'  Offset: {self.offset} RPM\n'
            f'  Publish rate: {self.publish_rate} Hz'
        )
    
    def timer_callback(self):
        """Generate and publish sine wave value"""
        elapsed_time = time.time() - self.start_time
        
        # Calculate sine wave: amplitude * sin(2*pi*f*t) + offset
        sine_value = self.amplitude * math.sin(2 * math.pi * self.frequency * elapsed_time) + self.offset
        
        msg = Float32()
        msg.data = sine_value
        
        self.publisher.publish(msg)
        
        self.get_logger().debug(f'Published: {sine_value:.2f} RPM')


def main(args=None):
    rclpy.init(args=args)
    node = SineWavePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
