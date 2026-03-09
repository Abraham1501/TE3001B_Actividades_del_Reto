"""
Start the Micro-ROS agent in a separate terminal:
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

Launch this file:
    ros2 launch motor_control pid_controller.launch.py

Set a velocity reference from another terminal:
       ros2 topic pub /cmd_vel_rpm std_msgs/msg/Float32 "data: 60.0"   # 60 RPM fwd
       ros2 topic pub /cmd_vel_rpm std_msgs/msg/Float32 "data: -40.0"  # 40 RPM rev
       ros2 topic pub /cmd_vel_rpm std_msgs/msg/Float32 "data: 0.0"    # stop

Launch arguments
kp          : Proportional gain   (default 0.55)
ki          : Integral gain       (default 2.0)
kd          : Derivative gain     (default 0.05)
rpm_max     : Maximum RPM         (default 110.0)
sample_time : Control period [s]  (default 0.1)

Example with custom gains:
    ros2 launch motor_control pid_controller.launch.py kp:=0.7 ki:=3.0 kd:=0.1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    kp_arg = DeclareLaunchArgument(
        'kp', default_value='0.55',
        description='PID proportional gain')

    ki_arg = DeclareLaunchArgument(
        'ki', default_value='2.0',
        description='PID integral gain')

    kd_arg = DeclareLaunchArgument(
        'kd', default_value='0.05',
        description='PID derivative gain')

    rpm_max_arg = DeclareLaunchArgument(
        'rpm_max', default_value='110.0',
        description='Maximum motor RPM')

    sample_time_arg = DeclareLaunchArgument(
        'sample_time', default_value='0.05',
        description='Control loop period in seconds (0.05 = 50 ms / 20 Hz)')

    pid_controller = Node(
        package='motor_control',
        executable='pid_velocity_controller',
        name='pid_velocity_controller',
        output='screen',
        parameters=[{
            'kp':          LaunchConfiguration('kp'),
            'ki':          LaunchConfiguration('ki'),
            'kd':          LaunchConfiguration('kd'),
            'rpm_max':     LaunchConfiguration('rpm_max'),
            'sample_time': LaunchConfiguration('sample_time'),
        }],
    )

    motor_monitor = Node(
        package='motor_control',
        executable='motor_monitor',
        name='motor_monitor',
        output='screen',
    )

    # Plot 1: tracking — setpoint vs actual RPM
    rqt_plot_tracking = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'rqt_plot', 'rqt_plot',
            '/pid/ref_rpm/data',
            '/motor/rpm/data',
        ],
        output='screen',
    )

    # Plot 2: applied effort — controller output % and raw PWM command
    rqt_plot_effort = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'rqt_plot', 'rqt_plot',
            '/pid/u_pct/data',
            '/cmd_pwm/data',
        ],
        output='screen',
    )

    rqt_graph = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_graph', 'rqt_graph'],
        output='screen',
    )

    # ── Launch description ────────────────────────────────────────────────────
    return LaunchDescription([
        kp_arg,
        ki_arg,
        kd_arg,
        rpm_max_arg,
        sample_time_arg,
        pid_controller,
        motor_monitor,
        # rqt_plot_tracking,
        # rqt_plot_effort,
        # rqt_graph,
    ])
