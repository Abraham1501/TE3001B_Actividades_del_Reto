# Arduino IDE Setup for Micro-ROS

This guide walks through configuring Arduino IDE for developing Micro-ROS applications on ESP32 (or Hackerboard) based on the official TE3001B course materials.

## Prerequisites

- Arduino IDE installed (download from [arduino.cc](https://www.arduino.cc/en/software))
- USB cable connected to ESP32/Hackerboard
- Ubuntu 22.04 with ROS 2 Humble installed
- Python `pyserial` module (required by ESP32 tools):
  ```bash
  pip3 install pyserial
  ```
  Or with sudo:
  ```bash
  sudo pip3 install pyserial
  ```

## Step 1: Install ESP32 Board Support

1. **Open Arduino IDE**

2. **Navigate to Board Manager**
   - Go to: **Tools** → **Board** → **Boards Manager**

3. **Search for ESP32**
   - In the search box, type: `esp32`

4. **Install ESP32 by Espressif**
   - Look for: **esp32 by Espressif Systems**
   - **IMPORTANT**: Select version **2.0.17** (NOT the newest!)
   - Reason: Newest versions are not fully compatible with Micro-ROS
   - Click **Install**
   - Wait for installation to complete (~2-3 minutes)

5. **Do NOT upgrade the library** - Keep it at 2.0.17

6. **Verify Installation**
   - Install an ESP32 example to test
   - Go to: **File** → **Examples** → **ESP32** → **Network** → **WiFiScan**
   - Select board: **Tools** → **Board** → **ESP32 Arduino** → **DOIT ESP32 DEVKIT V1**
   - Click the checkmark (✓) to compile
   - Should compile without errors

## Step 2: Install Micro-ROS Arduino Library

1. **Download Micro-ROS Arduino Package**
   - Go to: https://github.com/micro-ROS/micro_ros_arduino/releases
   - Find version: **v2.0.7-humble** (stable version)
   - Download: **Source code.zip**
   - Save to a known location (e.g., Downloads folder)

2. **Add Library to Arduino IDE**
   - Open Arduino IDE
   - Go to: **Sketch** → **Include Library** → **Add .ZIP Library**
   - Navigate to and select: `micro_ros_arduino-2.0.7-humble.zip`
   - Click **Open**
   - Wait for extraction (~1-2 minutes)

3. **Restart Arduino IDE**
   - Close and reopen Arduino IDE completely

4. **Verify Micro-ROS Installation**
   - Go to: **File** → **Examples** → **micro_ros_arduino**
   - You should see examples like:
     - `micro_ros_publisher`
     - `micro_ros_subscriber`
     - `publisher_subscriber`
     - etc.

5. **Test Compilation**
   - Open: **File** → **Examples** → **micro_ros_arduino** → **micro_ros_publisher**
   - Select board: **Tools** → **Board** → **ESP32 Arduino** → **DOIT ESP32 DEVKIT V1**
   - Click the checkmark (✓) to compile
   - Should compile without errors

## Step 3: Configure Serial Port (Ubuntu)

1. **Check Available Ports**
   ```bash
   ls /dev/tty*
   ```
   Look for `/dev/ttyUSB0` or `/dev/ttyACM0`

2. **Grant Port Permissions**
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   sudo chmod 666 /dev/ttyACM0
   ```

3. **Make Permissions Permanent** (Recommended)
   ```bash
   sudo usermod -a -G dialout $USER
   ```
   Then log out and log back in

4. **Verify in Arduino IDE**
   - Connect ESP32 via USB
   - Go to: **Tools** → **Port**
   - Should see `/dev/ttyUSB0` or `/dev/ttyACM0`
   - Select it

## Step 4: Configure ESP32 Board Settings

Before uploading any sketch, set these board parameters:

| Setting | Value |
|---------|-------|
| Board | DOIT ESP32 DEVKIT V1 |
| Upload Speed | 115200 |
| CPU Frequency | 240 MHz |
| Flash Frequency | 80 MHz |
| Flash Mode | DIO |
| Flash Size | 4MB (32Mb) |
| Partition Scheme | Huge APP (3MB No OTA/1MB SPIFFS) |
| Core Debug Level | Verbose |

**Where to set these:**
- **Tools** → [Setting Name] → [Value]

## Step 5: Test with Motor Node Example

Once everything is installed:

1. **Open the motor_node sketch**
   - Location: `ros2_ws/src/motor_control/arduino/motor_node.ino`
   - Or copy the code from the repository

2. **Select the board and port**
   - **Tools** → **Board** → **DOIT ESP32 DEVKIT V1**
   - **Tools** → **Port** → **/dev/ttyUSB0**

3. **Compile to verify**
   - Click the checkmark (✓)
   - Should compile without errors

4. **Upload to ESP32**
   - Click the arrow (→) to upload
   - Watch the serial output
   - Should see: `Setup complete. Waiting for ROS 2 agent...`

## Troubleshooting

### Error: "ModuleNotFoundError: No module named 'serial'"
This error happens during compilation when esptool (the ESP32 upload tool) tries to access the serial port.

**Fix:**
```bash
pip3 install pyserial
```
Then restart Arduino IDE and try again.

### Error: "Board not found"
- Make sure ESP32 is selected: **Tools** → **Board** → **DOIT ESP32 DEVKIT V1**
- Ensure USB cable is connected

### Error: "Port not available"
- Check port exists: `ls /dev/tty*`
- Grant permissions: `sudo chmod 666 /dev/ttyUSB0`
- Restart Arduino IDE
- Try a different USB port

### Error: "esp32 by Espressif" not found
- Make sure you searched for "esp32" (not "ESP32")
- Install version 2.0.17 specifically
- Do not upgrade after installation

### Error: "micro_ros_arduino examples not showing"
- Make sure you restarted Arduino IDE after adding the library
- Check that .zip file was fully extracted
- Try removing and reinstalling the library

### Compilation errors with Micro-ROS
- Ensure you have the correct board selected: **DOIT ESP32 DEVKIT V1**
- Check that CPU frequency is set to **240 MHz**
- Make sure ESP32 board version is **2.0.17** (not newer)

### Upload fails or gets stuck
- Try a different USB port
- Use a different USB cable (some are power-only)
- Increase upload speed: **Tools** → **Upload Speed** → **921600**
- Press and hold the **BOOT** button during upload

## Important Notes

⚠️ **Version Critical Points**:
- ESP32 Board: Must be **2.0.17** (not newest!)
- Micro-ROS Arduino: Must be **v2.0.7-humble** (matches ROS 2 Humble)
- Arduino IDE: 2.0.x or later recommended

## Next Steps

1. After successful upload, run the Micro-ROS agent on Ubuntu:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
   ```

2. Monitor motor feedback:
   ```bash
   ros2 run motor_control motor_monitor
   ```

3. Send motor commands:
   ```bash
   ros2 run motor_control motor_commander -- --pwm 150
   ```

## References

- Arduino IDE Documentation: https://docs.arduino.cc/
- Micro-ROS Arduino GitHub: https://github.com/micro-ROS/micro_ros_arduino
- ESP32 Official Documentation: https://docs.espressif.com/projects/esp-idf/
- Course Materials: `TE3001B_Intelligent_Robotics_Implementation_2026/Week 3/`
