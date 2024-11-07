##launch file
from launch import LaunchDescription
import launch_ros.actions 

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='nav_algo',
            executable='nav_algo_node.py',
            output='screen'
            ),
        ])

## to launch:
## ros2 launch nav_algo nav_algo.launch.py