# Motor Node Implementation for TE3001B Challenge

This document describes the Micro-ROS Motor Node that controls a DC motor connected to an ESP32 (or Hackerboard) via an L298N motor driver.

## Architecture Overview

The motor control system consists of three components:

### 1. **Motor Node** (Arduino/Micro-ROS on ESP32)
- **File**: `ros2_ws/src/motor_control/arduino/motor_node.ino`
- **Node Name**: `/motor_node`
- **Purpose**: Runs on the microcontroller to control the motor hardware
- **Subscribes To**: `/cmd_pwm` (Int16: -255 to +255)
- **Publishes**:
  - `/motor/rpm` (Float32): Current motor speed in RPM
  - `/motor/encoder` (Int32): Encoder pulse count
  - `/motor/state` (Int16): Motor state (0=stopped, 1=running)

### 2. **Motor Commander** (Python ROS 2 on Ubuntu)
- **File**: `ros2_ws/src/motor_control/motor_control/motor_commander.py`
- **Node Name**: `/motor_commander`
- **Purpose**: Simple test node to publish motor commands
- **Usage**:
  ```bash
  ros2 run motor_control motor_commander -- --pwm 150
  ```

### 3. **Motor Monitor** (Python ROS 2 on Ubuntu)
- **File**: `ros2_ws/src/motor_control/motor_control/motor_monitor.py`
- **Node Name**: `/motor_monitor`
- **Purpose**: Displays real-time motor feedback
- **Usage**:
  ```bash
  ros2 run motor_control motor_monitor
  ```

## Hardware Connections

Based on your circuit diagram and Arduino code:

| Component | GPIO | Signal Type | Purpose |
|-----------|------|-------------|---------|
| **L298N Motor Driver** | | | |
| - Direction Pin 1 | GPIO 26 (IN1) | Output | Set motor direction |
| - Direction Pin 2 | GPIO 25 (IN2) | Output | Set motor direction |
| - PWM (ENA) | GPIO 27 | PWM Output | Control motor speed |
| **Encoder** | | | |
| - Phase A | GPIO 18 | Input + Interrupt | Speed feedback |
| - Phase B | GPIO 19 | Input | Direction detection |
| **Status** | | | |
| - LED | GPIO 2 | Output | Status indicator (toggles each cycle) |

## Motor Command Mapping

### Command Range: -255 to +255

- **Positive values (1 to 255)**: Forward direction
  - IN1 = HIGH, IN2 = LOW
  - PWM = command value

- **Negative values (-255 to -1)**: Reverse direction
  - IN1 = LOW, IN2 = HIGH
  - PWM = absolute value of command

- **Zero (0)**: Stop/Brake
  - IN1 = HIGH, IN2 = HIGH
  - PWM = 0

### Example Commands

```
cmd_pwm = 255   → Max forward speed
cmd_pwm = 128   → Half forward speed
cmd_pwm = 50    → Low forward speed
cmd_pwm = 0     → Stop (braked)
cmd_pwm = -150  → Reverse at 150/255 speed
cmd_pwm = -255  → Max reverse speed
```

## Control Loop Details

The motor node runs a control loop at **100ms intervals**:

1. **Reads** encoder values (interrupt-safe)
2. **Calculates** current RPM from pulse count
3. **Publishes** feedback (RPM, encoder count, motor state)
4. **Applies** the latest PWM command from `/cmd_pwm`
5. **Toggles** LED for status indication

### Encoder Configuration
- **Pulses per revolution**: 495
- **Measurement window**: 100ms
- **RPM calculation**: `(pulses * 60) / (495 * time_seconds)`

## Installation & Setup

### Prerequisites

1. Micro-ROS agent installed on Ubuntu (use the setup script):
   ```bash
   ../setup_microros.sh
   ```

2. Arduino IDE with Micro-ROS libraries configured (see [MICROROS_SETUP.md](MICROROS_SETUP.md))

### Step 1: Upload Arduino Code to ESP32

1. Open Arduino IDE
2. Open the sketch: `ros2_ws/src/motor_control/arduino/motor_node.ino`
3. Select ESP32 board and COM port
4. Click **Upload**

The code initializes and waits for the ROS 2 agent to connect.

### Step 2: Start Micro-ROS Agent

```bash
source ~/Documents/classes/IRS_6to/ROS/TE3001B_Actividades_del_Reto/ros2_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

Wait for the agent to be ready. ESP32 will show as "connected" once agent is running.

### Step 3: Build ROS 2 Workspace

In a new terminal:

```bash
cd ~/Documents/classes/IRS_6to/ROS/TE3001B_Actividades_del_Reto/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/local_setup.bash
```

## Testing & Operation

### Test 1: Monitor Motor Feedback

```bash
ros2 run motor_control motor_monitor
```

You should see output like:
```
[Motor Status] State: STOPPED | RPM:    0.00 | Encoder:       0
[Motor Status] State: RUNNING | RPM:   45.32 | Encoder:     234
```

### Test 2: Command Motor (Forward)

In another terminal:
```bash
ros2 run motor_control motor_commander -- --pwm 150
```

You should see:
- Motor spinning forward
- RPM increasing in monitor
- Encoder count increasing

### Test 3: Command Motor (Stop)

```bash
ros2 run motor_control motor_commander -- --pwm 0
```

Motor should stop immediately.

### Test 4: Command Motor (Reverse)

```bash
ros2 run motor_control motor_commander -- --pwm -100
```

Motor should spin in reverse.

### Test 5: View All Topics

```bash
ros2 topic list
```

Expected topics:
- `/cmd_pwm` - Motor command input
- `/motor/rpm` - Current RPM output
- `/motor/encoder` - Encoder count output
- `/motor/state` - Motor state output

### Test 6: Inspect Topic Data

```bash
ros2 topic echo /motor/rpm
ros2 topic echo /motor/state
```

## Code Structure

### Motor Node (Arduino)

**Key Functions:**

1. **`isrEncoderA()`** - Interrupt handler for encoder phase A
   - Detects quadrature direction
   - Increments/decrements encoder count
   - Called on every phase A change

2. **`cmd_pwm_callback()`** - Receives motor commands from ROS 2
   - Constrains to valid range (-255 to +255)
   - Updates `currentPwmCommand`

3. **`control_timer_callback()`** - Main control loop (100ms)
   - Reads encoder atomically (disables interrupts)
   - Calculates RPM
   - Publishes feedback
   - Applies motor command

4. **`apply_motor_command()`** - Translates PWM to GPIO signals
   - Handles direction control (IN1/IN2 pins)
   - Applies PWM duty cycle
   - Manages motor stop condition

5. **State Machine** - Manages Micro-ROS connection
   - `WAITING_AGENT`: Searching for ROS 2 agent
   - `AGENT_AVAILABLE`: Agent found
   - `AGENT_CONNECTED`: Successfully connected and operating
   - `AGENT_DISCONNECTED`: Lost connection (stops motor and reconnects)

## Troubleshooting

### Motor doesn't respond to commands

1. Check motor power connections
2. Verify ESP32 is connected to Ubuntu via USB
3. Check if micro_ros_agent is running
4. Check if motor_node sketch compiled and uploaded successfully
5. Monitor with `motor_monitor` to see if commands are being received

**Verify:**
```bash
# Check if topics exist
ros2 topic list | grep motor

# Check if data is being published
ros2 topic echo /motor/rpm
```

### Encoder not working (RPM shows 0)

1. Check encoder phase A and B connections (GPIO 18, 19)
2. Verify encoder wiring is correct
3. Check interrupt is enabled and phase A on rising edge
4. Try rotating motor by hand while monitoring RPM

### Motor runs but stops suddenly

1. Check USB connection - may be getting disconnected
2. Check if agent is still running
3. Look for ROS 2 timeout or disconnection messages
4. Try re-uploading the sketch

### Serial port permission denied

```bash
# Grant permissions and add user to dialout group
sudo chmod a+rw /dev/ttyUSB0
sudo usermod -a -G dialout $USER

# Log out and log back in
```

## ROS 2 Message Specifications

### `/cmd_pwm` (Subscriber - Input)

**Type**: `std_msgs/Int16`

```yaml
data: 150   # Range: -255 (reverse) to +255 (forward)
```

### `/motor/rpm` (Publisher - Output)

**Type**: `std_msgs/Float32`

```yaml
data: 45.32  # Current motor speed in RPM
```

### `/motor/encoder` (Publisher - Output)

**Type**: `std_msgs/Int32`

```yaml
data: 1234   # Total encoder pulse count
```

### `/motor/state` (Publisher - Output)

**Type**: `std_msgs/Int16`

```yaml
data: 0   # 0=stopped, 1=running
```

## Implementation Details

### Why Micro-ROS?

- Allows ESP32 to communicate with ROS 2 system over serial
- Smaller memory footprint than full ROS 2
- Uses special serialization for constrained devices
- Bridges gap between microcontroller and Ubuntu workstation

### Threading Behavior

- No multithreading on ESP32 (single core)
- Encoder ISR is atomic and very fast
- ROS 2 callbacks executed from main loop
- 100ms control loop prevents blocking operations

### PWM Configuration

- **Frequency**: 100 Hz (standard for motor control)
- **Resolution**: 8-bit (0-255 range)
- **Channel**: 0
- **API**: `ledcSetup()`, `ledcAttachPin()`, `ledcWrite()`

## Extending the Code

### Adding PID Control (Optional)

You could enhance the node to include closed-loop PI control like your original code:

1. Store previous RPM and error
2. In control loop, calculate error between command and actual RPM
3. Compute PI correction to PWM
4. Apply corrected PWM instead of direct mapping

### Publishing Additional Telemetry

Add more publishers for:
- Motor current (if you have a current sensor)
- Motor temperature
- Command acceptance status
- Debug information

### Motor Safety Features

Consider adding:
- Command timeout (stop motor if no command received for N seconds)
- Maximum acceleration ramp
- Stall detection
- Temperature limits

## Files Created/Modified

```
ros2_ws/
├── src/
│   ├── motor_control/                 ← NEW: Motor control package
│   │   ├── motor_control/
│   │   │   ├── __init__.py
│   │   │   ├── motor_commander.py     ← Command publisher node
│   │   │   └── motor_monitor.py       ← Feedback monitor node
│   │   ├── arduino/
│   │   │   └── motor_node.ino         ← Micro-ROS Arduino sketch
│   │   ├── test/                      ← Unit tests
│   │   ├── resource/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   └── signal_proc/
│       ├── signal_gen.py
│       ├── signal_proc.py
│       └── setup.py
```

## Key Learning Points

1. **Interrupts**: Encoder ISR must be fast and atomic
2. **Edge Cases**: Handle encoder wraparound and direction changes
3. **ROS 2 Patterns**: State machine for connection management
4. **Micro-ROS Specifics**: Different message initialization than standard ROS 2
5. **Motor Control**: Direction/PWM mapping and locking mechanism

## References

- Micro-ROS Documentation: https://micro.ros.org/
- L298N Motor Driver Datasheet
- ESP32 PWM Documentation: https://docs.espressif.com/projects/esp-idf/en/release-v5.0/esp32/api-reference/peripherals/ledc.html
- Course materials: `TE3001B_Intelligent_Robotics_Implementation_2026/Week 3/`
