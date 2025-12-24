import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros

def generate_launch_description():
    robot_pkg_package_path = get_package_share_directory("robot_pkg")
    nav2_bringup_package_path = get_package_share_directory("nav2_bringup")
    rviz_config_path = os.path.join(nav2_bringup_package_path,"rviz","nav2_default_view.rviz")

    # 创建Launch参数配置
    use_sim_time = launch.substitutions.LaunchConfiguration(
        "use_sim_time", default="true")
    map_yaml_file = launch.substitutions.LaunchConfiguration(
        "map",
        default=os.path.join(robot_pkg_package_path, "maps", "room.yaml"))
    nav2_params_file = launch.substitutions.LaunchConfiguration(
        "params_file",
        default=os.path.join(robot_pkg_package_path, "config", "nav2_params.yaml"))
    
    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument("use_sim_time",default_value=use_sim_time,
                                                 description='Use simulation time in gazebo'),
            launch.actions.DeclareLaunchArgument("map",default_value=map_yaml_file,
                                                 description='Full path to map file to load'),
            launch.actions.DeclareLaunchArgument("params_file",default_value=nav2_params_file,
                                                 description='Full path to the ROS2 parameters file to use for all launched nodes'),
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [nav2_bringup_package_path, "/launch", "/bringup_launch.py"]),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                        "map": map_yaml_file,
                        "params_file": nav2_params_file
                    }.items(),
            ),
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_path],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen"
            ),
        ]
    )
