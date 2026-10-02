import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg = get_package_share_directory('explore_bot')
    slam_share = get_package_share_directory('slam_toolbox')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, 'launch', 'online_async_launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': os.path.join(pkg, 'config', 'slam_params.yaml'),
            }.items(),
        ),
    ])