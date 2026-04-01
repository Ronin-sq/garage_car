import launch
import launch.launch_description_sources
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import TimerAction
# from launch.substitutions import Command


def generate_launch_description():
    package_path = get_package_share_directory("robot_pkg")
    rebar_sim_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [package_path, '/launch', '/rebar_sim.launch.py']
        ),)
    

    scan_node = Node(
        package='algorithm_pkg',
        executable='scan_node',
        name='scan_node',
        output='screen',
    )

    path_visualization_node = Node(
        package='algorithm_pkg',
        executable='nav_path_node',
        name='nav_path_node',
        output='screen',
    )



    action_client_node = Node(
        package='algorithm_pkg',
        executable='action_client',
        name='action_client',
        output='screen',
    )

    action_server_node = Node(
        package='algorithm_pkg',
        executable='action_server',
        name='action_server',
        output='screen',
    )

    # Rosbridge节点 - 用于连接Web可视化工具(如 rosbridge_server + webvideobridge)
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        arguments=['--port', '9090'],
    )



    return launch.LaunchDescription([
        rebar_sim_launch,
        TimerAction(period=5.0, actions=[
            scan_node,
            path_visualization_node,
            action_client_node,
            # rosbridge_node,
            # action_server_node,
        ]),
    ])