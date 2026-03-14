"""
Clock Position Controller
=

The clock face is mapped to the motor encoder as follows:

    PULSES_PER_REV  = 495   (one full revolution of the shaft / clock hand)
    PULSES_PER_HOUR = 495 / 12  ≈ 41.25 pulses
    PULSES_PER_HALF = 495 / 24  ≈ 20.625 pulses


# Start at 12, go directly to 3 o'clock
ros2 topic pub /clock_cmd std_msgs/msg/Float32 "data: 3.0"

# Go to 6:30 on the first revolution
ros2 topic pub /clock_cmd std_msgs/msg/Float32 "data: 6.5"

# Do 10 full revolutions, then stop at 5 o'clock
ros2 topic pub /clock_cmd std_msgs/msg/Float32 "data: 125.0"

# Reset to home (12 o'clock, encoder 0 assumed as reference)
ros2 topic pub /clock_cmd std_msgs/msg/Float32 "data: 0.0"

# Hard reset current home from terminal
ros2 topic pub --once /clock_reset std_msgs/msg/Bool "{data: true}"
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32




def _hours_to_clock_face(hours: float) -> float:
    """Map cumulative hours to a position on the clock face [0.0, 12.0)."""
    return math.fmod(hours, 12.0)


def _face_to_hhmm(face_hours: float) -> str:
    """Return a human-readable clock string, e.g. '05:30'."""
    total_minutes = round(face_hours * 60.0)
    h = (total_minutes // 60) % 12
    m = total_minutes % 60
    label_h = 12 if h == 0 else h
    return f"{label_h:02d}:{m:02d}"




class ClockPositionController(Node):
    """
    Converts clock-face hours into absolute encoder targets for the
    motor_position_node running on the ESP32.
    """

    def __init__(self):
        super().__init__('clock_position_controller')

        # ---- Parameters ----
        self.declare_parameter('pulses_per_rev',   495.0)
        self.declare_parameter('tolerance_pulses',  10)
        self.declare_parameter('sample_time',        0.1)

        ppr  = self.get_parameter('pulses_per_rev').value
        tol  = self.get_parameter('tolerance_pulses').value
        ts   = self.get_parameter('sample_time').value

        self._pulses_per_rev   = float(ppr)
        self._pulses_per_hour  = self._pulses_per_rev / 12.0
        self._pulses_per_half  = self._pulses_per_rev / 24.0
        self._tolerance        = int(tol)

        # ---- State ----
        self._target_hours   = 0.0   # commanded cumulative hours
        self._current_pulses = 0     # latest encoder reading
        self._pos_error      = 0.0   # latest position error [pulses]
        self._reached_logged = True  # suppress repeated "reached" logs

        # ---- Subscribers ----
        self.create_subscription(Float32, 'clock_cmd',       self._clock_cmd_cb,    10)
        self.create_subscription(Bool,    'clock_reset',     self._clock_reset_cb,  10)
        self.create_subscription(Int32,   'motor/encoder',   self._encoder_cb,      10)
        self.create_subscription(Float32, 'motor/pos_error', self._pos_error_cb,    10)

        # ---- Publishers ----
        self._cmd_pos_pub     = self.create_publisher(Int32,   'cmd_pos',             10)
        self._tgt_hours_pub   = self.create_publisher(Float32, 'clock/target_hours',  10)
        self._cur_hours_pub   = self.create_publisher(Float32, 'clock/current_hours', 10)
        self._face_pos_pub    = self.create_publisher(Float32, 'clock/face_position', 10)

        # ---- Status timer ----
        self._timer = self.create_timer(ts, self._status_timer_cb)

        self.get_logger().info('Clock Position Controller started.')
        self.get_logger().info(
            f'  pulses/rev={self._pulses_per_rev:.1f}  '
            f'pulses/hour={self._pulses_per_hour:.3f}  '
            f'tolerance={self._tolerance} pulses')
        self.get_logger().info('')
        self.get_logger().info('  Send a target via:')
        self.get_logger().info(
            '    ros2 topic pub /clock_cmd std_msgs/msg/Float32 "data: <hours>"')
        self.get_logger().info(
            '    ros2 topic pub --once /clock_reset std_msgs/msg/Bool "{data: true}"')
        self.get_logger().info('')
        self.get_logger().info('  Formula:  hours = N_revolutions x 12 + hour_mark')
        self.get_logger().info('  Example:  10 turns + 5 pm  →  10x12 + 5 = 125.0')
        self.get_logger().info('')
        self.get_logger().info('  Supported positions (first revolution):')
        for h in range(13):
            pulses = int(round(h * self._pulses_per_hour))
            label  = 12 if h == 0 or h == 12 else h
            self.get_logger().info(f'    {label:2d}:00  →  {pulses:5d} pulses  (clock_cmd = {float(h):.1f})')
        self.get_logger().info(
            f'  Half-hour offset = {self._pulses_per_half:.3f} pulses  '
            f'(add 0.5 to clock_cmd)')


    def _clock_cmd_cb(self, msg: Float32):
        """Receive a new clock-face target in cumulative hours and send it."""
        hours = float(msg.data)
        if hours < 0.0:
            self.get_logger().warn(f'Negative clock_cmd ({hours:.3f} h) ignored. '
                                   'Only forward rotation is supported.')
            return

        self._target_hours   = hours
        self._reached_logged = False

        target_pulses = int(round(hours * self._pulses_per_hour))

        face   = _hours_to_clock_face(hours)
        label  = _face_to_hhmm(face)
        full_turns = int(hours // 12)

        self.get_logger().info(
            f'New target: {hours:.2f} h  →  {target_pulses} pulses  '
            f'({full_turns} full rev + {label})')

        cmd = Int32()
        cmd.data = target_pulses
        self._cmd_pos_pub.publish(cmd)

        t = Float32()
        t.data = float(hours)
        self._tgt_hours_pub.publish(t)

    def _encoder_cb(self, msg: Int32):
        self._current_pulses = msg.data

    def _clock_reset_cb(self, msg: Bool):
        """Reset local controller state when /clock_reset is asserted."""
        if not msg.data:
            return

        self._target_hours = 0.0
        self._current_pulses = 0
        self._pos_error = 0.0
        self._reached_logged = True

        cmd = Int32()
        cmd.data = 0
        self._cmd_pos_pub.publish(cmd)

        t = Float32()
        t.data = 0.0
        self._tgt_hours_pub.publish(t)

        cur_msg = Float32()
        cur_msg.data = 0.0
        self._cur_hours_pub.publish(cur_msg)

        face_msg = Float32()
        face_msg.data = 0.0
        self._face_pos_pub.publish(face_msg)

        self.get_logger().info('Reset received: home position set to 12:00 (encoder = 0).')

    def _pos_error_cb(self, msg: Float32):
        self._pos_error = msg.data

        # Log once when the motor reaches the target
        if not self._reached_logged and abs(self._pos_error) <= self._tolerance:
            face  = _hours_to_clock_face(self._target_hours)
            label = _face_to_hhmm(face)
            turns = int(self._target_hours // 12)
            self.get_logger().info(
                f'★  Position reached:  {label}  '
                f'(after {turns} full rev,  encoder = {self._current_pulses})')
            self._reached_logged = True


    # Status timer

    def _status_timer_cb(self):
        """Publish current position topics periodically."""
        current_hours = self._current_pulses / self._pulses_per_hour
        face_hours    = _hours_to_clock_face(current_hours)

        cur_msg  = Float32(); cur_msg.data  = float(current_hours)
        face_msg = Float32(); face_msg.data = float(face_hours)

        self._cur_hours_pub.publish(cur_msg)
        self._face_pos_pub.publish(face_msg)

    # Convenience helper (callable from other nodes or tests)

    def hours_to_pulses(self, total_hours: float) -> int:
        """Convert cumulative hours to absolute encoder-pulse target."""
        return int(round(total_hours * self._pulses_per_hour))

    def clock_position_to_hours(self, n_revolutions: int, hour: int,
                                 half: bool = False) -> float:
        """
        Build the clock_cmd value from human-readable inputs.

        Parameters
        ----------
        n_revolutions : int   – number of complete clock turns before stopping
        hour          : int   – destination hour mark (1-12, where 12 == 0 h)
        half          : bool  – True to stop at the half-hour mark

        Returns
        -------
        float  – value to publish on /clock_cmd

        Example
        -------
        # "10 turns then 5 pm"
        val = node.clock_position_to_hours(10, 5)   # → 125.0
        """
        hour_value = 0.0 if hour == 12 else float(hour)
        if half:
            hour_value += 0.5
        return float(n_revolutions) * 12.0 + hour_value


# Entry point

def main(args=None):
    rclpy.init(args=args)
    node = ClockPositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
