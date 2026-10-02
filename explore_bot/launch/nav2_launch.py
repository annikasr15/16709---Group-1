"""
Assignment 3, Task 4 Part B -- THE GRADED FILE. Assemble the components your
mobile_manipulator needs for autonomous navigation in the arena.

Your robot is spawned (via spawn_and_bridge.launch.py, already provided
and not part of this task), publishing odometry and a 320-degree lidar
scan. That alone is not autonomy. To actually navigate to a commanded
goal, your stack needs FOUR capabilities:

  (a) POSITION ESTIMATION -- something that figures out where the robot
      actually is in the environment, given that raw wheel odometry drifts
      and is not trustworthy on its own.

  (b) MAPPING -- something that provides the environment's layout (walls,
      obstacles) to whatever needs it. A correct map of this exact arena
      is already sitting in maps/map.yaml -- you are not building one live.

  (c) PLANNING -- something that computes a route from the robot's current
      position to a commanded goal, avoiding known obstacles.

  (d) SAFE EXECUTION -- something that turns that route into real-time
      velocity commands, continuously checking against what the lidar is
      currently seeing so it doesn't drive into something the static map
      didn't know about.

Go read the Nav2 architecture documentation and work out which package(s)
provide each of these. TOOLBOX.md in this directory has the generic
launch-file MECHANICS you'll need (how to declare a lifecycle node, how a
lifecycle_manager brings a group of nodes up, how to point a node at a
params file, how to include another launch file) -- it does not tell you
which components to use.

Everything you assemble needs correct lifecycle bring-up (autostart) and
should use config/nav2_params.yaml and maps/map.yaml, both already
provided and pre-tuned for this arena.

check_nav_launch.py (in code/) grades this behaviorally: it doesn't care
what shape your launch file takes, only that the right capabilities end up
active and that the robot actually reaches the fixed goal from the
handout.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

#amcl for map > odom
#load maps
#nav2 planner server for planning
#nav2 controller server for controller

def generate_launch_description():
    # TODO: assemble your autonomy stack here.

    # discover (relevant) files 
    pkg_share = get_package_share_directory('mobile_manipulator_description')
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_yaml_path = os.path.join(pkg_share, 'maps', 'map.yaml')


    # map_server_node = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[{'yaml_filename': map_yaml_path}]
    # )

    # # for localization
    # amcl_node = Node(
    #     package='nav2_amcl',
    #     executable='amcl',
    #     name='amcl',
    #     output='screen',
    #     parameters=[params_file]
    # )

    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file]
    )

    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file]
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file]
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
        }]
    )

    return LaunchDescription([
        # your components go here
        map_server_node, 
        amcl_node,
        planner_node,
        controller_node,
        behavior_node,
        bt_navigator_node,
        lifecycle_manager_localization,
        lifecycle_manager_navigation
    ])
