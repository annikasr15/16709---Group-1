"""Bring up spawn + SLAM + Nav2 + RViz, then start explore_lite once Nav2 is active.

Place in <your_pkg>/launch/bringup_launch.py and change PKG below to your package name.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

PKG = 'explore_bot'  # <-- the package that holds your launch/ and rviz/ folders
OTHER_PKG = 'explore_lite'


def generate_launch_description():
    pkg_share = get_package_share_directory(PKG)
    other_pkg_share = get_package_share_directory(OTHER_PKG)

    launch_dir = os.path.join(pkg_share, 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')
    explore_params = LaunchConfiguration('explore_params')

    # ---------------------------------------------------------------- arguments
    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_share, 'rviz', 'nav_explore.rviz'),
            description='RViz config loaded at startup'),
        DeclareLaunchArgument(
            'explore_params',
            default_value=os.path.join(other_pkg_share, 'config', 'params_costmap.yaml'),
            description='Parameter file for explore_lite'),
    ]

    # ------------------------------------------------------- the three includes
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'spawn_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav2_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    # SetParameter inside the group applies use_sim_time to every node launched
    # in it (including nodes in the included files that don't set it themselves).
    core_group = GroupAction([
        SetParameter(name='use_sim_time', value=use_sim_time),
        spawn,
        slam,
        nav2,
    ])

    # -------------------------------------------------------------------- RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    # ------------------------------------- wait for Nav2, then start explore_lite
    # Nav2 has no "finished launching" event, so poll until bt_navigator (the last
    # lifecycle node the nav2 lifecycle_manager activates) reports "active".
    wait_for_nav2 = ExecuteProcess(
        cmd=['bash', '-c',
             'until ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "^active"; '
             'do sleep 1; done; echo "[bringup] Nav2 is active"'],
        output='screen')

    explore = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[explore_params, {'use_sim_time': use_sim_time}])

    start_explore_after_nav2 = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_nav2,
            on_exit=[LogInfo(msg='Nav2 active -> starting explore_lite'), explore]))

    return LaunchDescription([
        *declare_args,
        core_group,
        rviz,
        wait_for_nav2,
        start_explore_after_nav2,
    ])