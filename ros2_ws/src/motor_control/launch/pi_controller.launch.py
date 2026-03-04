"""
Start the Micro-ROS agent in a separate terminal:
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

Launch this file:
    ros2 launch motor_control pi_controller.launch.py

Set a velocity reference from another terminal:
       ros2 topic pub /cmd_vel_rpm std_msgs/msg/Float32 "data: 60.0"   # 60 RPM fwd
       ros2 topic pub /cmd_vel_rpm std_msgs/msg/Float32 "data: -40.0"  # 40 RPM rev
       ros2 topic pub /cmd_vel_rpm std_msgs/msg/Float32 "data: 0.0"    # stop

Launch arguments
kp          : Proportional gain   (default 0.55)
ki          : Integral gain       (default 2.0)
rpm_max     : Maximum RPM         (default 110.0)
sample_time : Control period [s]  (default 0.1)

Example with custom gains:
    ros2 launch motor_control pi_controller.launch.py kp:=0.7 ki:=3.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    kp_arg = DeclareLaunchArgument(
        'kp', default_value='0.55',
        description='PI proportional gain')

    ki_arg = DeclareLaunchArgument(
        'ki', default_value='2.0',
        description='PI integral gain')

    rpm_max_arg = DeclareLaunchArgument(
        'rpm_max', default_value='110.0',
        description='Maximum motor RPM')

    sample_time_arg = DeclareLaunchArgument(
        'sample_time', default_value='0.1',
        description='Control loop period in seconds (0.1 = 100 ms)')


    pi_controller = Node(
        package='motor_control',
        executable='pi_velocity_controller',
        name='pi_velocity_controller',
        output='screen',
        parameters=[{
            'kp':          LaunchConfiguration('kp'),
            'ki':          LaunchConfiguration('ki'),
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
            '/pi/ref_rpm/data',
            '/motor/rpm/data',
        ],
        output='screen',
    )

    # Plot 2: applied effort — controller output % and raw PWM command
    rqt_plot_effort = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'rqt_plot', 'rqt_plot',
            '/pi/u_pct/data',
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
        rpm_max_arg,
        sample_time_arg,
        pi_controller,
        motor_monitor,
        # rqt_plot_tracking,
        # rqt_plot_effort,
        # rqt_graph,
    ])
