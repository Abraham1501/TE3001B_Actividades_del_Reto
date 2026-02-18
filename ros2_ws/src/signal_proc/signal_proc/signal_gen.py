# Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import time

#Class Definition
class SignalGenerator(Node):
    def __init__(self):
        super().__init__('signal_gen')

        # Publish result to "/signal"
        self.signal_pub = self.create_publisher(Float32, '/signal', 10)
        
        # Publish time t to "/time"
        self.time_pub = self.create_publisher(Float32, '/time', 10)
        
        # Use a rate of 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = time.time()
        
        self.get_logger().info('Signal Generator Node Started')

    def timer_callback(self):
        current_time = time.time()
        t = current_time - self.start_time
        
        # Generate sine wave y = sin(t)
        signal_value = math.sin(t)
        
        # Create messages
        sig_msg = Float32()
        sig_msg.data = signal_value
        
        time_msg = Float32()
        time_msg.data = t
        
        # Publish
        self.signal_pub.publish(sig_msg)
        self.time_pub.publish(time_msg)
        
        # Print result to terminal
        self.get_logger().info(f'Published -> Time: {t:.2f} | Signal: {signal_value:.2f}')

#Main
def main(args=None):
    rclpy.init(args=args)
    node = SignalGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

#Execute Node
if __name__ == '__main__':
    main()
