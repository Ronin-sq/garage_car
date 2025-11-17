import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import Command
import os

def generate_launch_description():
    robot_description_path = (get_package_share_directory("robot_pkg")+"/sdf/robot_car.sdf")
    rviz_config_path = (get_package_share_directory("robot_pkg")+"/config/rviz_config.rviz")
    urdf_path = (get_package_share_directory("robot_pkg")+"/urdf/robot_car.xacro")
    xacro_path = os.path.join(get_package_share_directory("robot_pkg"), "urdf", "robot_car_new.xacro")
    with open(xacro_path, 'r') as infp:
        robot_desc = infp.read()
        
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        
        parameters=[{'robot_description': Command(['xacro ', xacro_path])}],
    )
        # 启动控制器管理器
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            os.path.join(get_package_share_directory("robot_pkg"), 'config', 'controller.yaml'),
        ],
        output='screen'
    )
    
    # 加载差速控制器
    spawn_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # 加载关节状态广播器
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
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
        controller_manager,
        spawn_controller,
        spawn_joint_state_broadcaster,
        rviz_node,
    ])