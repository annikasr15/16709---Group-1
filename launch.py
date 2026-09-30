"""
Launch the final project world and spawn swebot into it. No ROS package needed.

Put this file in the same folder as your other files:
    16709---Group-1/
        arena.launch.py           <- this file
        arena_final_project.sdf
        urdf/swebot.urdf
        meshes/...

Run with:
    source /opt/ros/jazzy/setup.bash
    ros2 launch ~/cmu/16709/Final_Project/16709---Group-1/arena.launch.py
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
WORLD_NAME = 'arena_final_project'  # must match <world name="..."> in the .sdf
ROBOT_NAME = 'swebot'


def generate_launch_description():
    with open(ROBOT_FILE) as f:
        robot_description = f.read()

    # The URDF's meshes use package://swebot/meshes/..., which Gazebo turns into
    # model://swebot/meshes/... and looks for a FOLDER NAMED "swebot" on
    # GZ_SIM_RESOURCE_PATH. This project folder isn't named swebot, so make a
    # hidden shortcut .gz_models/swebot -> this folder and point Gazebo there.
    gz_models = os.path.join(PROJECT, '.gz_models')
    os.makedirs(gz_models, exist_ok=True)
    swebot_link = os.path.join(gz_models, 'swebot')
    if not os.path.lexists(swebot_link):
        os.symlink(PROJECT, swebot_link)

    gz_launch_path = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    return LaunchDescription([
        # WSL graphics fix
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),

        # Where Gazebo looks for the robot's mesh files
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_models),

        # 1. Start Gazebo with the world ('-r' = start running, not paused)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': f'-r {WORLD_FILE}',
                'on_exit_shutdown': 'True'
            }.items(),
        ),

        # 2. Publish the robot's description and joint transforms to ROS
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            output='screen'
        ),

        # 3. Spawn the robot at the center of the room, just above the floor
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

        # 4. Bridge Gazebo topics to ROS 2.
        #    '[' = Gazebo -> ROS (sensors), ']' = ROS -> Gazebo (commands).
        #    These names must match the plugins in swebot.urdf
        #    (DiffDrive: cmd_vel/odom/tf, gpu_lidar: scan, imu sensor: imu).
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