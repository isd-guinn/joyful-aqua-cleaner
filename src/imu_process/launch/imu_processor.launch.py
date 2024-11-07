##launch file
from launch import LaunchDescription
import launch_ros.actions 

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='imu_process',
            executable='processor',
            output='screen'
            ),
        ])

## to launch:
## ros2 launch imu_process imu_processor.launch.py