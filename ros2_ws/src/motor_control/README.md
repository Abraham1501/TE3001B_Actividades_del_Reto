# Motor Control Package

Micro-ROS based motor control package for the TE3001B Challenge. This package provides ROS 2 nodes for controlling a DC motor with the L298N motor driver connected to an ESP32 (or Hackerboard) microcontroller.

## Quick Start

### 1. Upload Arduino Sketch to ESP32

Open Arduino IDE and upload `arduino/motor_node.ino` to your ESP32 board.

### 2. Build the Package

```bash
cd ~/Documents/classes/IRS_6to/ROS/TE3001B_Actividades_del_Reto/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select motor_control
source install/local_setup.bash
```

### 3. Start Micro-ROS Agent

```bash
source ~/Documents/classes/IRS_6to/ROS/TE3001B_Actividades_del_Reto/ros2_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

### 4. Monitor Motor Feedback

```bash
ros2 run motor_control motor_monitor
```

### 5. Send Motor Commands

In another terminal:

```bash
# Forward at 150/255 speed
ros2 run motor_control motor_commander -- --pwm 150

# Stop
ros2 run motor_control motor_commander -- --pwm 0

# Reverse at 100/255 speed
ros2 run motor_control motor_commander -- --pwm -100
```

## Package Contents

- **`motor_control/motor_commander.py`** - Publisher node for motor PWM commands
- **`motor_control/motor_monitor.py`** - Subscriber node for monitoring motor feedback
- **`arduino/motor_node.ino`** - Micro-ROS Arduino sketch for ESP32
- **`package.xml`** - ROS 2 package metadata
- **`setup.py`** - Python package configuration

## Topics

### Subscription
- **`/cmd_pwm`** (std_msgs/Int16): Motor command (-255 to +255)

### Publishing
- **`/motor/rpm`** (std_msgs/Float32): Motor speed in RPM
- **`/motor/encoder`** (std_msgs/Int32): Encoder pulse count
- **`/motor/state`** (std_msgs/Int16): Motor state (0=stopped, 1=running)

## Documentation

See [MOTOR_NODE.md](../MOTOR_NODE.md) in the repository root for complete documentation.

## Hardware Requirements

- ESP32 or Hackerboard microcontroller
- L298N motor driver module
- DC motor with encoder (495 pulses/rev)
- USB cable for serial communication

## GPIO Configuration

| Component | GPIO |
|-----------|------|
| Motor IN1 | 26 |
| Motor IN2 | 25 |
| Motor PWM | 27 |
| Encoder A | 18 |
| Encoder B | 19 |
| LED Status | 2 |
