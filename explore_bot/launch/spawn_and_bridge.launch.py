"""
Assignment 3, Task 4 -- PROVIDED, not graded. Starts the arena, spawns your
mobile_manipulator.urdf into it, bridges cmd_vel/odom/tf/scan/clock between
ROS 2 and Gazebo, and seeds AMCL's initial pose (the robot always spawns at
the same known point, x=-2.0 y=-2.0 yaw=0, so this is safe to automate).

This is the mechanical plumbing -- your Task 4 Part B work is the SEPARATE
autonomy_bringup launch file (map serving / localization / planning), not
this one.

Usage:
    ros2 launch mobile_manipulator_description spawn_and_bridge.launch.py \
        urdf_file:=/absolute/path/to/your/mobile_manipulator.urdf
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction, ExecuteProcess, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("mobile_manipulator_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    urdf_file_arg = DeclareLaunchArgument(
        "urdf_file",
        description="Absolute path to your completed mobile_manipulator.urdf "
                    "(base template + your Task 1 arm spliced in + both "
                    "Gazebo plugin snippets pasted in)",
    )
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="true = server only, no GUI (used for automated/CI "
                    "runs; on your own machine leave this false so you can "
                    "see the simulation for your video)",
    )

    # Your arm's meshes use package://arm_description/meshes/*.stl, which
    # sdformat rewrites to model://arm_description/... Gazebo resolves that
    # by searching GZ_SIM_RESOURCE_PATH for a directory named
    # "arm_description", so each entry must be the directory CONTAINING a
    # package's share dir, not the share dir itself.
    resource_paths = [
        os.path.dirname(get_package_share_directory("arm_description")),
        os.path.dirname(pkg_share),
    ]
    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if existing:
        resource_paths.append(existing)
    set_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.pathsep.join(resource_paths))

    world_path = os.path.join(pkg_share, "worlds", "arena.sdf")
    gz_flags = PythonExpression(
        ["'-r -s ' if '", LaunchConfiguration("headless"), "' == 'true' else '-r '"])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": [gz_flags, world_path]}.items(),
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file", LaunchConfiguration("urdf_file"),
            "-name", "mobile_manipulator",
            "-x", "-2.0", "-y", "-2.0", "-z", "0.06",
        ],
        output="screen",
    )

    # The lidar's <gz_frame_id> makes gz stamp /scan with "lidar_link".
    # Nothing here runs robot_state_publisher (the model is spawned straight
    # from file, and DiffDrive only publishes odom->base_link), so
    # base_link -> lidar_link comes from here instead -- the same fixed
    # offsets as lidar_joint in the URDF template. Without it AMCL and the
    # costmaps cannot relate scan data to base_link.
    lidar_frame_fix = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0.13", "--y", "0", "--z", "0.10",
            "--frame-id", "base_link",
            "--child-frame-id", "lidar_link",
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        output="screen",
    )

    # Seed AMCL's initial pose. Belt-and-suspenders alongside the
    # set_initial_pose/initial_pose params in config/nav2_params.yaml --
    # this works regardless of which AMCL version honors those params, as
    # long as AMCL is already active by the time it fires (hence the delay).
    seed_initial_pose = TimerAction(
        period=8.0,
        actions=[ExecuteProcess(
            cmd=[
                "ros2", "topic", "pub", "--once", "/initialpose",
                "geometry_msgs/msg/PoseWithCovarianceStamped",
                "{header: {frame_id: map}, pose: {pose: {position: "
                "{x: -2.0, y: -2.0, z: 0.0}, orientation: {w: 1.0}}, "
                "covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, "
                "0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06853891945200942]}}",
            ],
            output="screen",
        )],
    )

    return LaunchDescription([
        set_resource_path,
        urdf_file_arg,
        headless_arg,
        gz_sim,
        spawn,
        bridge,
        lidar_frame_fix,
        seed_initial_pose,
    ])
