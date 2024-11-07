##launch file
from launch import LaunchDescription
import launch_ros.actions 

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='uart_slave',
            executable='m2s_sender',
            output='screen'
            ),
        launch_ros.actions.Node(
            package='uart_slave',
            executable='s2m_receiver',
            output='screen'
            ),
        launch_ros.actions.Node(
            package='uart_slave',
            executable='c2m_receiver',
            output='screen'
            ),
        ])

## to launch:
## ros2 launch uart_slave uart_slave.launch.py
