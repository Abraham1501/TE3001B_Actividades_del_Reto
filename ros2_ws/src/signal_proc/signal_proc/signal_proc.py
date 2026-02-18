# Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from collections import deque

class SignalProcessing(Node):
    def __init__(self):
        super().__init__('signal_proc')

        # Subscribe to "/signal"
        self.signal_sub = self.create_subscription(
            Float32, '/signal', self.signal_callback, 10
        )

        # Publisher
        self.proc_pub = self.create_publisher(Float32, '/proc_signal', 10)

        # ===== PARAMETERS =====
        self.sample_rate = 10.0      
        self.amplitude_factor = 0.5   
        self.offset = 0.0 

        delay_time = 3.141592653589793 / 2.0
        self.N_delay = int(round(delay_time * self.sample_rate))

        self.delay_line = deque(maxlen=1000)

        self.get_logger().info(
            f'Signal Processing Node Started | Delay Samples: {self.N_delay}'
        )

    def signal_callback(self, msg):
        x = msg.data

        self.delay_line.append(x)

        if len(self.delay_line) <= self.N_delay:
            return

        x_delayed = self.delay_line[-(self.N_delay + 1)]

        y = -x_delayed

        y = self.amplitude_factor * y + self.offset

        # Publicar
        out_msg = Float32()
        out_msg.data = y
        self.proc_pub.publish(out_msg)

        self.get_logger().info(
            f'Input: {x:.2f} | Cosine-ish: {y:.2f}'
        )

# Main
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

if __name__ == '__main__':
    main()