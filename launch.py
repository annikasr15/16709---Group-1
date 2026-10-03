"""
Launch the final project world and spawn swebot into it.

Run with:
    source /opt/ros/jazzy/setup.bash
    ros2 launch ~/cmu/16709/Final_Project/swebot/launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Everything lives in the same folder as this launch file
PROJECT = os.path.dirname(os.path.abspath(__file__))
WORLD_FILE = os.path.join(PROJECT, 'arena_final_project.sdf')
ROBOT_FILE = os.path.join(PROJECT, 'urdf', 'swebot.urdf')
WORLD_NAME = 'arena_final_project'  
ROBOT_NAME = 'swebot'


def generate_launch_description():
    with open(ROBOT_FILE) as f:
        robot_description = f.read()

    gz_launch_path = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    return LaunchDescription([
        
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),

        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(PROJECT)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': f'-r {WORLD_FILE}',
                'on_exit_shutdown': 'True'
            }.items(),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', WORLD_NAME,
                '-topic', 'robot_description',
                '-name', ROBOT_NAME,
                '-x', '0', '-y', '0', '-z', '0.02',
            ],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
            output='screen'
        ),
    ])