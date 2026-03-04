# TE3001B Actividades del Reto

This repository contains the activities for the TE3001B challenge, along with a submodule referencing the official course material.

## Submodule: TE3001B_Intelligent_Robotics_Implementation_2026

The folder `TE3001B_Intelligent_Robotics_Implementation_2026` is a git submodule pointing to:
[https://github.com/ManchesterRoboticsLtd/TE3001B_Intelligent_Robotics_Implementation_2026](https://github.com/ManchesterRoboticsLtd/TE3001B_Intelligent_Robotics_Implementation_2026)

### First-time setup

After cloning this repository, initialize and pull the submodule:

```bash
git submodule update --init --recursive
```

## Workspace: ros2_ws

The `ros2_ws` directory contains the ROS 2 workspace with packages for the challenge activities.

### Micro-ROS Setup

To set up Micro-ROS for ESP32 microcontroller communication, run the automated setup script:

```bash
./setup_microros.sh
```

This script:
- Installs required dependencies (vcstool, rosdep packages)
- Builds the ROS 2 workspace
- Creates and builds the Micro-ROS agent from the submodule
- Configures serial port permissions

For detailed setup instructions, see [MICROROS_SETUP.md](MICROROS_SETUP.md)

### Arduino IDE Setup

After setting up the Micro-ROS agent, configure Arduino IDE with Micro-ROS libraries for ESP32:

👉 **See [ARDUINO_IDE_SETUP.md](ARDUINO_IDE_SETUP.md)** for complete instructions

Key steps:
1. Install ESP32 board support (version 2.0.17)
2. Add Micro-ROS Arduino library (v2.0.7-humble)
3. Configure serial port permissions
4. Upload sketches to ESP32

### Motor Control Package

The `ros2_ws/src/motor_control` package provides Micro-ROS-based DC motor control:

- **Subscribe to** `/cmd_pwm` for motor commands (-255 to +255)
- **Publish** motor feedback: `/motor/rpm`, `/motor/encoder`, `/motor/state`
- Works with ESP32/Hackerboard + L298N motor driver

**Quick usage:**
```bash
# Upload arduino/motor_node.ino to ESP32 (see ARDUINO_IDE_SETUP.md)
# Start ROS 2 agent and then:
ros2 run motor_control motor_commander -- --pwm 150  # Forward
ros2 run motor_control motor_monitor                  # Monitor feedback
```

See [MOTOR_NODE.md](MOTOR_NODE.md) and [motor_control README](ros2_ws/src/motor_control/README.md) for details.

### Updating the submodule to the latest version

Since the course material is updated regularly, follow these steps to pull the latest changes:

1. Enter the submodule directory and fetch + checkout the desired commit:

```bash
cd TE3001B_Intelligent_Robotics_Implementation_2026
git fetch origin
git checkout <commit-hash>   # or a branch, e.g. "main"
cd ..
```

2. Stage the updated submodule reference in this repo:

```bash
git add TE3001B_Intelligent_Robotics_Implementation_2026
```

3. Commit the change:

```bash
git commit -m "Update submodule to latest version"
```

4. Push:

```bash
git push
```

> The parent repository only stores a pointer (commit hash) to the submodule. Every time the course material is updated, repeat the steps above to advance that pointer to the new commit.
