import launch
import launch.launch_description_sources
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command


def generate_launch_description():

    package_path = get_package_share_directory("robot_pkg")

    ronin_robot_file = os.path.join(package_path, "urdf", "robot","robot.urdf.xacro")
    world_file = os.path.join(package_path, "world", "sim.world")
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', ronin_robot_file])}],
    )
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py']),
        launch_arguments=[('world', world_file),('verbose', 'true')],
        )
    
    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py", 
        arguments=["-topic", "robot_description", "-entity", "ronin_robot"],
    
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(package_path, 'config', 'robot.rviz')],
        output='screen',
    )

    return launch.LaunchDescription([
        robot_state_publisher_node,
        action_launch_gazebo,
        spawn_entity_node,
        rviz_node,
    ])