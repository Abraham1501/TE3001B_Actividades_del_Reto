from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Node Signal Generator
        Node(
            package='signal_proc',
            executable='signal_gen',
            name='signal_gen',
            output='screen'
        ),
        
        # Node Process Node
        Node(
            package='signal_proc',
            executable='signal_proc',
            name='signal_proc',
            output='screen'
        ),
        
        # RQT Plot
        # /signal/data and /proc_signal/data
        Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='rqt_plot',
            # Pass topics as arguments to the node
            arguments=['/signal/data', '/proc_signal/data'],
            output='screen'
        )
    ])