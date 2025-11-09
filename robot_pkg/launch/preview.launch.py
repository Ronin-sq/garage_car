import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    robot_description_path = (get_package_share_directory("robot_pkg")+"/sdf/robot_car.sdf")
    rviz_config_path = (get_package_share_directory("robot_pkg")+"/config/rviz_config.rviz")
    urdf_path = (get_package_share_directory("robot_pkg")+"/urdf/robot_car.urdf")
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
        
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )
    
    # Check if RVIZ config file exists
    if os.path.exists(rviz_config_path):
        rviz_node = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
        )
    else:
        # Launch RVIZ2 without config file
        rviz_node = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
        )
        
    return launch.LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
    ])