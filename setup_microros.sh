#!/bin/bash

# Micro-ROS Setup Script
# This script sets up the micro-ROS agent in the ros2_ws workspace

set -e  # Exit on error

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$SCRIPT_DIR/ros2_ws"

echo "================================"
echo "Micro-ROS Agent Setup"
echo "================================"

# Check if ROS 2 Humble is installed
if [ ! -d "/opt/ros/humble" ]; then
    echo "Error: ROS 2 Humble is not installed!"
    exit 1
fi

# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Install dependencies
echo "Installing dependencies..."
sudo apt update -y
sudo apt install python3-vcstool -y
sudo rosdep update || true

# Install rosdep dependencies
echo "Installing rosdep dependencies..."
cd "$WS_DIR"
rosdep install --from-paths src --ignore-src -y || true

# Build the workspace
echo "Building ros2_ws..."
cd "$WS_DIR"
colcon build

# Create the micro-ROS agent workspace
echo "Creating micro-ROS agent workspace..."
source "$WS_DIR/install/local_setup.sh"
ros2 run micro_ros_setup create_agent_ws.sh

# Build the micro-ROS agent
echo "Building micro-ROS agent..."
source "$WS_DIR/install/local_setup.sh"
ros2 run micro_ros_setup build_agent.sh

# Set serial port permissions
echo "Configuring serial port permissions..."
sudo chmod a+rw /dev/tty* 2>/dev/null || true
sudo usermod -a -G dialout $USER

echo ""
echo "================================"
echo "Setup Complete!"
echo "================================"
echo ""
echo "To use the micro-ROS agent, run:"
echo "  source ~/Documents/classes/IRS_6to/ROS/TE3001B_Actividades_del_Reto/ros2_ws/install/local_setup.bash"
echo "  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0"
echo ""
echo "Note: You may need to log out and log back in for serial port permissions to take effect."
echo ""
