import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_path = get_package_share_directory("robot_pkg")
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(package_path, 'config', 'ekf.yaml')]
    )

    return launch.LaunchDescription([
        ekf_node
    ])
