"""
Run the full exploration test from 5 hard-coded start positions, one after another.

For each scenario:
  1. starts Gazebo with swebot at that scenario's position, then SLAM, Nav2,
     and explore_lite (same stack as bringup_launch.py),
  2. waits until explore_lite reports there are no frontiers left,
  3. prints "Scenario N complete", runs the metrics script (while the simulation
     and live /map are still up) and prints its output, then saves the map as
     scenario_N_map.pgm/.yaml for keeping,
  4. shuts everything down and starts the next scenario.
After scenario 5 it prints "All scenarios have been run".

Saved maps and full launch logs go to ~/scenario_logs/<date_time>/.

Run with:
    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws/install/setup.bash
    python3 ~/ros2_ws/src/explore_bot/launch/run_scenarios_launch.py

How it works: this one file is both the test runner (bottom of the file, used
when you run it with python3) and the launch file for a single scenario
(generate_launch_description). The runner launches this same file once per
scenario and tells it which scenario to run through the SCENARIO_INDEX
environment variable. Nothing needs to be passed in by hand.
"""
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (ExecuteProcess, GroupAction, IncludeLaunchDescription,
                            LogInfo, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter

# =============================================================================
# SETTINGS
# =============================================================================

# Start positions (x, y, yaw) in the Gazebo world frame: meters, meters, radians.
# PLACEHOLDERS - replace with your 5 positions.
SCENARIOS = [
    (0.0, 0.0, 0.0),     # scenario 1
    (1.0, -3.5, 0.0),    # scenario 2
    (-2.0, 2.0, 0.0),    # scenario 3
    (2.0, 2.0, 0.0),     # scenario 4
    (-2.0, -2.0, 0.0),   # scenario 5
]

# True = no Gazebo window and no RViz (faster). False = show both.
HEADLESS = True

# Metrics script. PLACEHOLDER - point this at your teammate's file.
# It runs right after each scenario finishes, while the simulation and map are
# still up. Everything it prints is shown in the terminal.
METRICS_COMMAND = ['python3', os.path.expanduser('~/ros2_ws/src/explore_bot/scripts/metrics.py')]

# explore_lite settings file. None = explore_lite's default (same as bringup_launch.py).
# To use your own (e.g. with a smaller min_frontier_size), put the full path, e.g.:
#   EXPLORE_PARAMS = os.path.expanduser('~/ros2_ws/src/explore_bot/config/explore_params.yaml')
EXPLORE_PARAMS = None

SCENARIO_TIMEOUT = 20 * 60   # seconds before a scenario is given up on
METRICS_TIMEOUT = 2 * 60     # seconds the metrics script may take
MAP_SAVE_TIMEOUT = 30        # seconds to wait for the map to be saved

# =============================================================================

PKG = 'explore_bot'
WORLD_NAME = 'arena'   # must match <world name="..."> in arena.sdf
ROBOT_NAME = 'swebot'
DONE_PHRASES = ('No frontiers found', 'Exploration stopped')


# -----------------------------------------------------------------------------
# Launch file for ONE scenario (used by `ros2 launch`, started by the runner)
# -----------------------------------------------------------------------------
def generate_launch_description():
    index = int(os.environ.get('SCENARIO_INDEX', '1'))
    x, y, yaw = SCENARIOS[index - 1]

    pkg_share = get_package_share_directory(PKG)
    launch_dir = os.path.join(pkg_share, 'launch')
    world_file = os.path.join(pkg_share, 'worlds', 'arena.sdf')
    with open(os.path.join(pkg_share, 'urdf', 'swebot.urdf')) as f:
        robot_description = f.read()

    gz_launch_path = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    gz_args = f'-r -s {world_file}' if HEADLESS else f'-r {world_file}'
    explore_params = EXPLORE_PARAMS or os.path.join(
        get_package_share_directory('explore_lite'), 'config', 'params_costmap.yaml')

    # Same as spawn_launch.py, but with this scenario's start position
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'True'}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', WORLD_NAME, '-topic', 'robot_description', '-name', ROBOT_NAME,
                   '-x', str(x), '-y', str(y), '-z', '0.1', '-Y', str(yaw)],
        output='screen')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen')

    # SLAM and Nav2: the team's existing launch files, unchanged
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items())
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav2_launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items())

    core_group = GroupAction([
        SetParameter(name='use_sim_time', value=True),
        gazebo, robot_state_publisher, spawn, bridge, slam, nav2,
    ])

    # Same as bringup_launch.py: start explore_lite once Nav2 is active
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
        parameters=[explore_params, {'use_sim_time': True}])

    actions = [
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg_share)),
        LogInfo(msg=f'Scenario {index}: spawning at x={x} y={y} yaw={yaw}'),
        core_group,
        wait_for_nav2,
        RegisterEventHandler(OnProcessExit(
            target_action=wait_for_nav2,
            on_exit=[LogInfo(msg='Nav2 active -> starting explore_lite'), explore])),
    ]

    if not HEADLESS:
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'explore.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen'))

    return LaunchDescription(actions)


# -----------------------------------------------------------------------------
# Test runner (used when this file is run with python3)
# -----------------------------------------------------------------------------
def _forward_output(proc, log_file, lines):
    """Copy the launch output into the log file and hand each line to the runner."""
    for line in proc.stdout:
        log_file.write(line)
        log_file.flush()
        lines.put(line)
    lines.put(None)  # launch has exited


def _stop(proc):
    """Ctrl+C the whole launch, then force it if it doesn't stop."""
    if proc.poll() is not None:
        return
    for sig, wait in ((signal.SIGINT, 20), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        try:
            os.killpg(proc.pid, sig)
            proc.wait(timeout=wait)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            continue


def _cleanup_leftovers():
    """Kill anything a previous run left behind so the next scenario starts clean."""
    for pattern in ('gz sim', 'parameter_bridge', 'robot_state_publisher',
                    'slam_toolbox', 'nav2', 'component_container',
                    'explore_lite', 'rviz2'):
        subprocess.run(['pkill', '-f', pattern],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(3)


def _save_map(n, log_dir):
    """Save the current SLAM map as scenario_N_map.yaml + .pgm, as a record.

    Not used by the metrics script (that reads /map live); kept for the report
    and for re-checking results later without rerunning the simulation.
    """
    base = os.path.join(log_dir, f'scenario_{n}_map')
    try:
        result = subprocess.run(
            ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', base,
             '--ros-args', '-p', 'use_sim_time:=true',
             '-p', f'save_map_timeout:={float(MAP_SAVE_TIMEOUT - 5)}'],
            capture_output=True, text=True, timeout=MAP_SAVE_TIMEOUT)
    except subprocess.TimeoutExpired:
        print(f'  Could not save the map: no map received within {MAP_SAVE_TIMEOUT}s.')
        return

    if result.returncode == 0 and os.path.exists(base + '.yaml'):
        print(f'  Map saved: {base}.pgm / .yaml')
    else:
        print('  Could not save the map:')
        print('  ' + (result.stderr or result.stdout).strip().replace('\n', '\n  '))


def _run_metrics(n, x, y, yaw, explore_seconds):
    """Run the metrics script and print whatever it outputs.

    It runs while the simulation is still up, so it can read the live map (/map).
    """
    env = dict(os.environ,
               SCENARIO_INDEX=str(n), START_X=str(x), START_Y=str(y), START_YAW=str(yaw),
               EXPLORE_WALL_TIME_S=f'{explore_seconds:.1f}')
    try:
        result = subprocess.run(METRICS_COMMAND, env=env, capture_output=True,
                                text=True, timeout=METRICS_TIMEOUT)
    except FileNotFoundError:
        print(f'  Metrics script not found: {METRICS_COMMAND}')
        return
    except subprocess.TimeoutExpired:
        print(f'  Metrics script took longer than {METRICS_TIMEOUT}s and was stopped.')
        return

    output = result.stdout.strip()
    print(output if output else '  (metrics script printed nothing)')
    if result.returncode != 0:
        print(f'  Metrics script exited with an error (code {result.returncode}):')
        print('  ' + result.stderr.strip().replace('\n', '\n  '))


def _run_scenario(n, log_dir):
    x, y, yaw = SCENARIOS[n - 1]
    print(f'\n=== Scenario {n}/{len(SCENARIOS)}: start at x={x}, y={y}, yaw={yaw} ===', flush=True)
    print('  Starting Gazebo, SLAM and Nav2...', flush=True)

    log_path = os.path.join(log_dir, f'scenario_{n}.log')
    log_file = open(log_path, 'w')
    proc = subprocess.Popen(
        ['ros2', 'launch', os.path.abspath(__file__)],
        # Unbuffered output so "No frontiers found" reaches the runner right away
        env=dict(os.environ, SCENARIO_INDEX=str(n), PYTHONUNBUFFERED='1',
                 RCUTILS_LOGGING_BUFFERED_STREAM='0'),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
        start_new_session=True)
    lines = queue.Queue()
    reader = threading.Thread(target=_forward_output, args=(proc, log_file, lines), daemon=True)
    reader.start()

    started = time.time()
    explore_started = None
    outcome = 'timed out'
    try:
        while time.time() - started < SCENARIO_TIMEOUT:
            try:
                line = lines.get(timeout=1)
            except queue.Empty:
                continue
            if line is None:
                outcome = 'launch stopped unexpectedly'
                break
            if explore_started is None and 'starting explore_lite' in line:
                explore_started = time.time()
                print('  Nav2 is up, exploring...', flush=True)
            if 'explore_node' in line and any(p in line for p in DONE_PHRASES):
                outcome = 'complete'
                break

        if outcome == 'complete':
            explore_seconds = time.time() - (explore_started or started)
            print(f'\nScenario {n} complete', flush=True)
            if explore_seconds < 15:
                print('  Warning: explore_lite stopped almost immediately - it may not '
                      'have explored at all.')
            _run_metrics(n, x, y, yaw, explore_seconds)
            _save_map(n, log_dir)
        else:
            print(f'\nScenario {n} did not complete: {outcome}. Log: {log_path}')
    finally:
        print('  Shutting down...', flush=True)
        _stop(proc)
        reader.join(timeout=5)
        log_file.close()
        _cleanup_leftovers()


def main():
    if 'ROS_DISTRO' not in os.environ:
        sys.exit('ROS is not sourced. Run: source /opt/ros/jazzy/setup.bash '
                 'and source ~/ros2_ws/install/setup.bash')

    log_dir = os.path.expanduser(os.path.join(
        '~', 'scenario_logs', datetime.now().strftime('%Y-%m-%d_%H-%M-%S')))
    os.makedirs(log_dir, exist_ok=True)
    print(f'Maps and launch logs will be saved in {log_dir}')

    _cleanup_leftovers()
    try:
        for n in range(1, len(SCENARIOS) + 1):
            _run_scenario(n, log_dir)
    except KeyboardInterrupt:
        print('\nStopped early (Ctrl+C).')
        _cleanup_leftovers()
        return

    print('\nAll scenarios have been run')
    print(f'Maps and logs: {log_dir}')


if __name__ == '__main__':
    main()