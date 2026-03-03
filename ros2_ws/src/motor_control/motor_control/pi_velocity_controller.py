import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16


class PIVelocityController(Node):
    """Closed-loop PI velocity controller for a DC motor."""

    def __init__(self):
        super().__init__('pi_velocity_controller')

        self.declare_parameter('kp',          0.55)
        self.declare_parameter('ki',          2.0)
        self.declare_parameter('rpm_max',   110.0)
        self.declare_parameter('sample_time', 0.1)   # seconds (100 ms)

        self.kp          = self.get_parameter('kp').value
        self.ki          = self.get_parameter('ki').value
        self.rpm_max     = self.get_parameter('rpm_max').value
        self.sample_time = self.get_parameter('sample_time').value

        self._ref_rpm    = 0.0   # reference set-point [RPM]  (+ fwd / - rev)
        self._actual_rpm = 0.0   # latest measured RPM from /motor/rpm
        self._u_prev     = 0.0   # previous control output [%]
        self._e_prev     = 0.0   # previous error [%]
        self._dir_prev   = None  # previous direction (True=fwd, False=rev)

        self.create_subscription(
            Float32, 'motor/rpm',    self._rpm_callback,  10)
        self.create_subscription(
            Float32, 'cmd_vel_rpm',  self._ref_callback,  10)

        self._cmd_pwm_pub  = self.create_publisher(Int16,   'cmd_pwm',   10)
        self._error_pub    = self.create_publisher(Float32, 'pi/error',  10)
        self._u_pct_pub    = self.create_publisher(Float32, 'pi/u_pct',  10)
        self._ref_rpm_pub  = self.create_publisher(Float32, 'pi/ref_rpm', 10)
        self._timer = self.create_timer(self.sample_time, self._control_loop)

        self.get_logger().info('PI Velocity Controller started.')
        self.get_logger().info(
            f'  Kp={self.kp}  Ki={self.ki}  RPM_max={self.rpm_max}  '
            f'Ts={self.sample_time*1000:.0f} ms')
        self.get_logger().info(
            '  Set reference  → ros2 topic pub /cmd_vel_rpm '
            'std_msgs/msg/Float32 "data: <rpm>"')


    def _rpm_callback(self, msg: Float32):
        """Receive measured RPM from the motor node (signed)."""
        self._actual_rpm = msg.data

    def _ref_callback(self, msg: Float32):
        """Receive RPM set-point.  Positive = forward, negative = reverse."""
        new_ref = float(msg.data)
        if new_ref != self._ref_rpm:
            self.get_logger().debug(f'Reference changed: {self._ref_rpm:.2f} → {new_ref:.2f} RPM')
        self._ref_rpm = new_ref


    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))


    def _control_loop(self):
        """Incremental PI controller, executed every sample_time seconds."""
        ref    = self._ref_rpm
        actual = self._actual_rpm
        ts     = self.sample_time

        direction_forward = (ref >= 0.0)
        control_pct = self._clamp(abs(ref)    * 100.0 / self.rpm_max, 0.0, 105.0)
        rpm_pct     = self._clamp(abs(actual) * 100.0 / self.rpm_max, 0.0, 130.0)

        # Consider reference zero when its magnitude is < 0.2 % of max RPM
        center_stop = control_pct < 0.2

        # Reset integrator when direction reverses to prevent wind-up
        if self._dir_prev is not None and direction_forward != self._dir_prev:
            self._u_prev = 0.0
            self._e_prev = 0.0
            self.get_logger().debug('Direction change detected – integrator reset')
        self._dir_prev = direction_forward

        cmd = Int16()

        if center_stop:
            # Stop and reset integrator
            cmd.data  = 0
            self._u_prev = 0.0
            self._e_prev = 0.0
            e_pct = 0.0
            u_pct = 0.0
        else:
            # Incremental (velocity) form:
            #   u(k) = u(k-1) + Kp*(e(k)-e(k-1)) + Ki*Ts*e(k)
            e_pct = control_pct - rpm_pct
            u_pct = (self._u_prev
                     + self.kp * (e_pct - self._e_prev)
                     + self.ki * ts * e_pct)
            u_pct = self._clamp(u_pct, 0.0, 100.0)

            # Convert percentage to 8-bit PWM duty cycle [1..255]
            pwm_duty = int(u_pct * 255.0 / 100.0 + 0.5)
            pwm_duty = int(self._clamp(float(pwm_duty), 1.0, 255.0))

            # Apply direction sign (-255..+255 as expected by motor_node)
            cmd.data = pwm_duty if direction_forward else -pwm_duty

            self._u_prev = u_pct
            self._e_prev = e_pct

        # ── Publish ───────────────────────────────────────────────────────────
        self._cmd_pwm_pub.publish(cmd)

        error_msg = Float32(); error_msg.data = float(e_pct if not center_stop else 0.0)
        u_msg     = Float32(); u_msg.data     = float(u_pct if not center_stop else 0.0)
        ref_msg   = Float32(); ref_msg.data   = float(ref)

        self._error_pub.publish(error_msg)
        self._u_pct_pub.publish(u_msg)
        self._ref_rpm_pub.publish(ref_msg)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PIVelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down PI Velocity Controller...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
