"""
Position Controller Launch File
================================
Starts the micro_ros_agent (serial transport) and the clock position controller.

Usage
-----
    ros2 launch motor_control position_controller.launch.py

    # Override serial port or baud rate:
    ros2 launch motor_control position_controller.launch.py \
        serial_port:=/dev/ttyUSB1 baud_rate:=921600

    # Tune controller parameters at launch:
    ros2 launch motor_control position_controller.launch.py \
        pulses_per_rev:=495.0 tolerance_pulses:=10

After launch, send a clock position command (cumulative hours from 12 o'clock):

    # Go to 3 o'clock (first revolution)
    ros2 topic pub /clock_cmd std_msgs/msg/Float32 "data: 3.0"

    # Do 10 full turns then stop at 5 pm
    ros2 topic pub /clock_cmd std_msgs/msg/Float32 "data: 125.0"

    # Half-hour positions: add 0.5 to the hour mark
    ros2 topic pub /clock_cmd std_msgs/msg/Float32 "data: 5.5"  # 5:30

Monitor feedback:
    ros2 topic echo /motor/encoder
    ros2 topic echo /motor/pos_error
    ros2 topic echo /clock/face_position
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ---- Launch Arguments ----
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port connected to the ESP32')

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for micro_ros_agent serial transport')

    pulses_per_rev_arg = DeclareLaunchArgument(
        'pulses_per_rev',
        default_value='495.0',
        description='Encoder pulses per revolution (must match firmware #define)')

    tolerance_pulses_arg = DeclareLaunchArgument(
        'tolerance_pulses',
        default_value='10',
        description='Position dead-band radius in encoder pulses')

    sample_time_arg = DeclareLaunchArgument(
        'sample_time',
        default_value='0.1',
        description='Status publish rate for the clock controller node [s]')

    # ---- Nodes ----
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=[
            'serial',
            '--dev',  LaunchConfiguration('serial_port'),
            '--baudrate', LaunchConfiguration('baud_rate'),
        ],
        output='screen',
    )

    clock_controller_node = Node(
        package='motor_control',
        executable='clock_position_controller',
        name='clock_position_controller',
        parameters=[{
            'pulses_per_rev':   LaunchConfiguration('pulses_per_rev'),
            'tolerance_pulses': LaunchConfiguration('tolerance_pulses'),
            'sample_time':      LaunchConfiguration('sample_time'),
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        pulses_per_rev_arg,
        tolerance_pulses_arg,
        sample_time_arg,
        # micro_ros_agent_node,
        clock_controller_node,
    ])
