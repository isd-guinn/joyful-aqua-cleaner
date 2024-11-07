##launch file
from launch import LaunchDescription
import launch_ros.actions 

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='serial_imu',
            executable='talker',
            output='screen'
            ),
        launch_ros.actions.Node(
            package='serial_imu',
            executable='listener',
            output='screen'
            ),
        ])

## to launch:
## ros2 launch serial_imu imu_spec_msg.launch.py
