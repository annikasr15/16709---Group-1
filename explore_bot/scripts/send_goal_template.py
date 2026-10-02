#!/usr/bin/env python3
"""
Assignment 3, Bonus C — send a fixed navigation goal from code.

Fill in the TODO below using nav2_simple_commander's BasicNavigator. You
are not choosing the goal coordinate -- it is fixed (see the handout) so
everyone's run is checkable against the same target.

Usage (after your Task 4 Part B autonomy stack is up and localized):
    python3 send_goal_template.py
"""

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

# Fixed goal -- see the handout for why this point is reachable.
GOAL_X = 2.0
GOAL_Y = 2.0
GOAL_YAW = 0.0  # radians


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # TODO: build a PoseStamped for (GOAL_X, GOAL_Y, GOAL_YAW) in the "map"
    # frame, wait for Nav2 to be active, send it with
    # navigator.goToPose(...), then poll navigator.isTaskComplete() /
    # navigator.getResult() until it finishes. Print the final result.
    #
    # Do NOT call navigator.lifecycleShutdown() -- that tears down the
    # whole Nav2 stack (AMCL, planner, everything), which your separately
    # launched autonomy bringup owns, not this script. Just clean up this
    # script's own node below.

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
