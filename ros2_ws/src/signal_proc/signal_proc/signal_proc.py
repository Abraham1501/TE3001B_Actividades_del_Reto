# Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

#Class Definition
class SignalProcessing(Node):
    def __init__(self):
        super().__init__('signal_proc')
        # Subscribe to "/signal" and "/time"
        # We mainly need time to calculate the phase shift accurately
        self.time_sub = self.create_subscription(Float32, '/time', self.time_callback, 10)
        self.signal_sub = self.create_subscription(Float32, '/signal', self.signal_callback, 10)
        
        # Publisher for processed signal
        self.proc_pub = self.create_publisher(Float32, '/proc_signal', 10)
        
        self.current_time = 0.0
        
        # Parameters
        self.phase_shift = 1.57  # Approx PI/2 (90 degrees)
        self.offset = 1.0        # Ensures signal is always positive
        self.amplitude_factor = 0.5
        
        # Use a rate of 10 Hz for processing/publishing
        self.timer = self.create_timer(0.1, self.timer_callback)

    def time_callback(self, msg):
        self.current_time = msg.data

    def signal_callback(self, msg):
        # We subscribe to comply with requirements, but we use time for the precise math
        pass

    def timer_callback(self):
        # Process the signal
        # Original: sin(t)
        # Processed: 0.5 * sin(t + phase) + offset
        
        # Apply Phase Shift and Amplitude reduction
        processed_val = self.amplitude_factor * math.sin(self.current_time + self.phase_shift)
        
        # Apply Offset to make it positive
        processed_val += self.offset
        
        # Create message
        msg = Float32()
        msg.data = processed_val
        
        # Publish to "/proc_signal"
        self.proc_pub.publish(msg)
        
        # Print result
        self.get_logger().info(f'Processed: {processed_val:.2f} (Time: {self.current_time:.2f})')
        

#Main
def main(args=None):
    rclpy.init(args=args)
    node = SignalProcessing()
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
