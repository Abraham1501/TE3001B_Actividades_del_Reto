# Motor Control Package

Micro-ROS based motor control package for the TE3001B Challenge. Provides ROS 2 nodes for
closed-loop PID speed control of a DC motor driven by an L298N module connected to an
ESP32 (or Hackerboard) microcontroller via Micro-ROS.

---

## Package Contents

| File | Description |
|------|-------------|
| `motor_control/pid_velocity_controller.py` | Closed-loop **PID** velocity controller node |
| `motor_control/pi_velocity_controller.py`  | Legacy PI velocity controller node |
| `motor_control/motor_monitor.py`           | Subscribes to motor feedback and prints it |
| `motor_control/motor_commander.py`         | One-shot PWM command publisher |
| `motor_control/sine_wave_publisher.py`     | Publishes a sine-wave RPM reference |
| `launch/pid_controller.launch.py`          | Launches the PID controller + motor monitor |
| `launch/pi_controller.launch.py`           | Legacy PI controller launch file |
| `arduino/motor_node.ino`                   | Micro-ROS sketch for the ESP32 |

---

## PID Velocity Controller

### What it does

`pid_velocity_controller` implements a **discrete incremental (velocity-form) PID** that
regulates motor speed in RPM.  The control law per sample is:

```
u(k) = u(k-1) + Kp*(e(k) - e(k-1))
               + Ki*Ts*e(k)
               + Kd/Ts*(e(k) - 2*e(k-1) + e(k-2))
```

where all errors are normalised to a percentage of `rpm_max` so that the gains are
dimensionless and independent of the hardware speed limit.

**Anti-windup / robustness features:**
- Output clamped to `[0 %, 100 %]` at every step.
- Integrator and derivative memory reset on direction reversal and when the reference is
  set to zero.

### Parameters

| Parameter     | Default | Description |
|---------------|---------|-------------|
| `kp`          | 0.55    | Proportional gain |
| `ki`          | 2.0     | Integral gain |
| `kd`          | 0.05    | Derivative gain |
| `rpm_max`     | 110.0   | Motor speed ceiling [RPM] |
| `sample_time` | 0.1     | Control-loop period [s] (100 ms) |

### Topics

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Subscribe | `/motor/rpm`    | `std_msgs/Float32` | Measured motor speed (signed) |
| Subscribe | `/cmd_vel_rpm`  | `std_msgs/Float32` | RPM set-point (+ fwd / − rev) |
| Publish   | `/cmd_pwm`      | `std_msgs/Int16`   | PWM command to motor (−255 … +255) |
| Publish   | `/pid/ref_rpm`  | `std_msgs/Float32` | Current RPM reference |
| Publish   | `/pid/u_pct`    | `std_msgs/Float32` | Controller output [%] |
| Publish   | `/pid/error`    | `std_msgs/Float32` | Tracking error [%] |

---

## How to Run

### 1. Upload the Arduino sketch

Open Arduino IDE and upload `arduino/motor_node.ino` to your ESP32 board.

### 2. Build the package

```bash
cd ~/Documents/classes/IRS_6to/ROS/TE3001B_Actividades_del_Reto/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select motor_control
source install/local_setup.bash
```

### 3. Start the Micro-ROS agent (dedicated terminal)

```bash
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

### 4. Launch the PID controller

```bash
ros2 launch motor_control pid_controller.launch.py
```

The launch file starts `pid_velocity_controller` and `motor_monitor` simultaneously.

#### Launch with custom gains

```bash
ros2 launch motor_control pid_controller.launch.py kp:=0.7 ki:=3.0 kd:=0.1
```

All five parameters (`kp`, `ki`, `kd`, `rpm_max`, `sample_time`) can be overridden on the
command line.

### 5. Send a velocity reference (another terminal)

```bash
# 60 RPM forward
ros2 topic pub /cmd_vel_rpm std_msgs/msg/Float32 "data: 60.0"

# 40 RPM reverse
ros2 topic pub /cmd_vel_rpm std_msgs/msg/Float32 "data: -40.0"

# Stop
ros2 topic pub /cmd_vel_rpm std_msgs/msg/Float32 "data: 0.0"
```

### 6. Optional — real-time plots with rqt_plot

Tracking (setpoint vs actual):
```bash
ros2 run rqt_plot rqt_plot /pid/ref_rpm/data /motor/rpm/data
```

Control effort:
```bash
ros2 run rqt_plot rqt_plot /pid/u_pct/data /cmd_pwm/data
```

---

## Live PID Tuning

Parameters can be changed **while the node is running** — no restart needed.

### Set a single gain

```bash
ros2 param set /pid_velocity_controller kp 0.7
ros2 param set /pid_velocity_controller ki 1.5
ros2 param set /pid_velocity_controller kd 0.0
```

Each change is logged by the node and takes effect on the very next control loop iteration.

### Check current values

```bash
ros2 param get /pid_velocity_controller kp
ros2 param get /pid_velocity_controller ki
ros2 param get /pid_velocity_controller kd
```

Or dump all at once:

```bash
ros2 param dump /pid_velocity_controller
```

### Tuning workflow

1. Launch the controller and open rqt_plot on `/pid/ref_rpm/data` and `/motor/rpm/data`.
2. Start with `kd=0` and `ki=0`. Raise `kp` until the response is fast but just starts to oscillate, then back off slightly.
3. Add `ki` gradually to eliminate steady-state error.
4. Add a small `kd` only if you need to reduce overshoot — it amplifies noise.

### Typical starting point

| Gain | Conservative | Aggressive |
|------|-------------|------------|
| `kp` | 0.4 | 0.8 |
| `ki` | 1.0 | 3.0 |
| `kd` | 0.0 | 0.05 |

---

## Hardware Requirements

- ESP32 or Hackerboard microcontroller
- L298N motor driver module
- DC motor with encoder (495 pulses/rev)
- USB cable for serial communication

## GPIO Configuration

| Component  | GPIO |
|------------|------|
| Motor IN1  | 26   |
| Motor IN2  | 25   |
| Motor PWM  | 27   |
| Encoder A  | 18   |
| Encoder B  | 19   |
| LED Status | 2    |



