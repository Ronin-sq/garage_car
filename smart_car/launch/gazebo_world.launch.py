from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包的share目录
    pkg_smart_car = get_package_share_directory('smart_car')
    
    # 定义world文件路径
    world_file_path = os.path.join(pkg_smart_car, 'world', 'garbage.world')
    
    # 声明launch参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 声明world参数
    world = LaunchConfiguration('world')
    
    # 声明launch参数
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=world_file_path,
        description='Full path to world model file to load')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    # Gazebo启动
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world}.items(),
    )
    
    # 创建launch描述
    ld = LaunchDescription()
    
    # 添加声明
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    
    # 添加gazebo节点
    ld.add_action(gazebo)
    
    return ld