# Micro-ROS Installation Guide

This document outlines how to set up Micro-ROS in the `ros2_ws` workspace for communicating with ESP32 microcontrollers.

## Overview

This installation has two main components:

1. **Micro-ROS Agent** - Runs on Ubuntu with ROS 2 Humble, acts as a bridge between the microcontroller and ROS 2
2. **Micro-ROS Libraries** (Arduino) - Configured separately in the Arduino IDE for ESP32 programming

## Quick Setup

If you've cloned this repository, run the automatic setup script:

```bash
cd /home/soni/Documents/classes/IRS_6to/ROS/TE3001B_Actividades_del_Reto
./setup_microros.sh
```

## Manual Setup (if needed)

### Prerequisites

- ROS 2 Humble installed (`/opt/ros/humble` should exist)
- Ubuntu 22.04 or similar
- `colcon` build tool

### Step 1: Initialize Submodule

The micro_ros_setup is included as a git submodule:

```bash
cd /home/soni/Documents/classes/IRS_6to/ROS/TE3001B_Actividades_del_Reto
git submodule update --init --recursive
```

### Step 2: Install Dependencies

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws

# Install vcstool for version control operations
sudo apt update -y
sudo apt install python3-vcstool -y

# Install rosdep dependencies
rosdep update
rosdep install --from-paths src --ignore-src -y
```

### Step 3: Build the Workspace

```bash
cd ros2_ws
colcon build
source install/local_setup.sh
```

### Step 4: Create Micro-ROS Agent Workspace

```bash
ros2 run micro_ros_setup create_agent_ws.sh
```

This will download the micro-ROS agent source code to `src/uros/`.

### Step 5: Build Micro-ROS Agent

```bash
source install/local_setup.sh
ros2 run micro_ros_setup build_agent.sh
```

This builds:
- **micro_ros_msgs** - ROS 2 message definitions
- **micro_ros_agent** - The serial bridge executable

### Step 6: Configure Serial Permissions

```bash
sudo chmod a+rw /dev/tty*
sudo usermod -a -G dialout $USER
```

Then log out and log back in.

## Using the Micro-ROS Agent

Once installed, run the agent with:

```bash
source ~/Documents/classes/IRS_6to/ROS/TE3001B_Actividades_del_Reto/ros2_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

The agent will wait for an ESP32 to connect on the serial port.

## Repository Structure

```
ros2_ws/
├── src/
│   ├── micro_ros_setup/       ← Submodule with setup scripts
│   ├── signal_proc/           ← Your signal processing package
│   ├── uros/                  ← Generated (ignored in git)
│   │   ├── micro-ROS-Agent/
│   │   └── micro_ros_msgs/
│   └── ros2.repos             ← Generated (ignored in git)
├── build/                     ← Generated (ignored in git)
├── install/                   ← Generated (ignored in git)
└── log/                       ← Generated (ignored in git)
```

## What Gets Version Controlled

Only committed to git:
- `src/micro_ros_setup` submodule reference
- The submodule branch configuration (`.gitmodules`)
- `src/signal_proc` package files
- Setup documents and scripts

Ignored by git (generated during setup):
- `src/uros/` - Downloaded Micro-ROS source
- `src/ros2.repos` - Repository manifest
- `build/`, `install/`, `log/` - Build artifacts

## Troubleshooting

### Error: "vcs: command not found"
Install vcstool: `sudo apt install python3-vcstool -y`

### Error: "ImportError: No module named serial"
Install pyserial: `pip3 install pyserial`

### Error: "python not found in $PATH"
Install: `sudo apt install python-is-python3`

### Serial port not recognized
- Check connections with: `ls /dev/tty*`
- For VM users, ensure USB device is passed through
- Try different USB ports or cables

## Next Steps

After the agent is installed, configure the Arduino IDE with Micro-ROS libraries for the ESP32. See the presentation in `TE3001B_Intelligent_Robotics_Implementation_2026/Week 3/Presentations/PDF/MCR2_Micro_ROS_Installation.pdf` for Arduino IDE configuration steps.

## References

- [Micro-ROS GitHub](https://github.com/micro-ROS)
- [Micro-ROS Documentation](https://micro.ros.org)
- Course material in `TE3001B_Intelligent_Robotics_Implementation_2026/Week 3/`
