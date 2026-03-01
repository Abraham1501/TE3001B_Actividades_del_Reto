from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([

        # Console monitor: prints RPM / encoder / state / cmd every 100 ms
        Node(
            package='motor_control',
            executable='motor_monitor',
            name='motor_monitor',
            output='screen',
        ),

        # rqt_plot: time-series plot of actual RPM vs reference PWM command
        # Topic paths use the /field suffix required by rqt_plot
        ExecuteProcess(
            cmd=['rqt_plot', '/motor/rpm/data', '/cmd_pwm/data'],
            output='screen',
        ),

        # rqt_graph: computation graph (nodes + topics) – useful for
        # verifying that motor_node, motor_monitor, and motor_commander
        # are all correctly connected
        ExecuteProcess(
            cmd=['rqt_graph'],
            output='screen',
        ),

    ])
