"""
Run the full exploration test from 5 hard-coded start positions, one after another.

For each scenario:
  1. starts Gazebo with swebot at that scenario's position, then SLAM, Nav2,
     and explore_lite (same stack as bringup_launch.py),
  2. waits until explore_lite reports there are no frontiers left,
  3. prints "Scenario N complete", saves the map as scenario_N_map.pgm/.yaml, then
     runs metrics.py on the live /map (simulation still up), prints the results
     and saves them as scenario_N_metrics.json,
  4. shuts everything down and starts the next scenario.
After scenario 5 it prints a summary table (also saved as summary.json) and
"All scenarios have been run".

Saved maps and full launch logs go to ~/scenario_logs/<date_time>/.

Run with:
    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws/install/setup.bash
    python3 ~/ros2_ws/src/explore_bot/launch/demo_launch.py

How it works: this one file is both the test runner (bottom of the file, used
when you run it with python3) and the launch file for a single scenario
(generate_launch_description). The runner launches this same file once per
scenario and tells it which scenario to run through the SCENARIO_INDEX
environment variable. Nothing needs to be passed in by hand.
"""
import os
import queue
import re
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
    (0.0, 0.0, 0.2),     # scenario 1
    (-3.0, 0, 0.2),    # scenario 2
    (2.0, 2.0, 0.2),    # scenario 3
    (2.0, -2.5, 0.2),     # scenario 4
    (-3.0, -2.0, 0.2),   # scenario 5
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
SETTLE_SECONDS = 4           # pause after 'done' so SLAM publishes its last map update

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
        #SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
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


def _sim_time_now(timeout=10):
    """Current Gazebo sim time in seconds, read from /clock (None if unavailable)."""
    try:
        out = subprocess.run(['ros2', 'topic', 'echo', '--once', '/clock'],
                             capture_output=True, text=True, timeout=timeout).stdout
        sec = int(re.search(r'sec:\s*(\d+)', out).group(1))
        nsec = int(re.search(r'nanosec:\s*(\d+)', out).group(1))
        return sec + nsec * 1e-9
    except Exception:
        return None


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
        return None

    if result.returncode == 0 and os.path.exists(base + '.yaml'):
        print(f'  Map saved: {base}.pgm / .yaml')
        return base + '.yaml'
    else:
        print('  Could not save the map:')
        print('  ' + (result.stderr or result.stdout).strip().replace('\n', '\n  '))
        return None


def _run_metrics(n, x, y, yaw, explore_seconds, sim_seconds, log_dir, map_yaml, completed):
    """Run the metrics script and print whatever it outputs.

    It scores the live /map (needs the simulation to still be up). The saved map
    (map_yaml) is passed along only as a fallback if the live map can't be read.
    """
    env = dict(os.environ,
               SCENARIO_INDEX=str(n), START_X=str(x), START_Y=str(y), START_YAW=str(yaw),
               EXPLORE_WALL_TIME_S=f'{explore_seconds:.1f}',
               EXPLORE_SIM_TIME_S=(f'{sim_seconds:.1f}' if sim_seconds is not None else ''),
               RESULTS_DIR=log_dir,
               COMPLETED='1' if completed else '0')
    if map_yaml:
        env['MAP_YAML'] = map_yaml
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
    sim_started = None
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
                sim_started = _sim_time_now()
                print('  Nav2 is up, exploring...', flush=True)
            if 'explore_node' in line and any(p in line for p in DONE_PHRASES):
                outcome = 'complete'
                break

        if outcome in ('complete', 'timed out'):
            explore_seconds = time.time() - (explore_started or started)
            sim_ended = _sim_time_now()  # must be read while the simulation is still up
            sim_seconds = (sim_ended - sim_started
                           if sim_started is not None and sim_ended is not None else None)
            if outcome == 'complete':
                print(f'\nScenario {n} complete', flush=True)
                if explore_seconds < 15:
                    print('  Warning: explore_lite stopped almost immediately - it may not '
                          'have explored at all.')
            else:
                print(f'\nScenario {n} timed out after {SCENARIO_TIMEOUT}s - '
                      'scoring the partial map', flush=True)
            time.sleep(SETTLE_SECONDS)
            map_yaml = _save_map(n, log_dir)
            _run_metrics(n, x, y, yaw, explore_seconds, sim_seconds, log_dir, map_yaml,
                         completed=(outcome == 'complete'))
        else:
            print(f'\nScenario {n} did not complete: {outcome}. Log: {log_path}')
    finally:
        print('  Shutting down...', flush=True)
        _stop(proc)
        reader.join(timeout=5)
        log_file.close()
        _cleanup_leftovers()


def _print_summary(log_dir):
    """Table of all scenarios + mean/std, saved as summary.json."""
    import glob
    import json
    results = []
    for path in sorted(glob.glob(os.path.join(log_dir, 'scenario_*_metrics.json'))):
        with open(path) as f:
            results.append(json.load(f))
    if not results:
        print('\nNo metrics were produced, so there is no summary.')
        return
    def fmt(v, spec):
        return '-' if v is None else format(v, spec)

    print('\n=== Summary ===')
    print(f"{'#':>2} {'done':>5} {'sim(s)':>8} {'real(s)':>8} {'cov%':>7} {'F1':>6} "
          f"{'err(cm)':>8} {'ffree%':>7} {'score':>6}")
    for r in results:
        print(f"{r['scenario']:>2} {'yes' if r.get('completed', True) else 'NO':>5} "
              f"{fmt(r.get('mapping_sim_time_s'), '.1f'):>8} "
              f"{fmt(r.get('mapping_wall_time_s'), '.1f'):>8} "
              f"{r['coverage'] * 100:>7.1f} {r['obstacle_f1']:>6.3f} "
              f"{r['mean_error_m'] * 100:>8.1f} {r['false_free_fraction'] * 100:>7.1f} "
              f"{r['score']:>6.1f}")
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
        from metrics import summarize
        summary = summarize(results)
    except Exception as e:  # numpy/scipy missing etc.
        print(f'  (could not compute mean/std: {e})')
        return
    import statistics
    for key in ('mapping_sim_time_s', 'mapping_wall_time_s'):
        vals = [r[key] for r in results if r.get(key) is not None]
        summary[key] = ({'mean': round(statistics.mean(vals), 1),
                         'std': round(statistics.pstdev(vals), 1),
                         'min': round(min(vals), 1), 'max': round(max(vals), 1)}
                        if vals else None)

    # (summary key, label, display scale, unit, decimals)
    rows = [('mapping_sim_time_s', 'Mapping time (sim)', 1, 's', 1),
            ('mapping_wall_time_s', 'Mapping time (real)', 1, 's', 1),
            ('coverage', 'Coverage', 100, '%', 1),
            ('obstacle_f1', 'Obstacle F1', 1, '', 3),
            ('mean_error_m', 'Mean error', 100, 'cm', 1),
            ('false_free_fraction', 'False-free', 100, '%', 1),
            ('score', 'Composite score', 1, '/ 100', 1)]
    print('\nAcross scenarios (mean +/- std, min - max):')
    for k, label, scale, unit, dec in rows:
        v = summary.get(k)
        if v:
            f = lambda x: f"{x * scale:.{dec}f}"
            print(f"  {label:<20} {f(v['mean'])} +/- {f(v['std'])} {unit}   "
                  f"({f(v['min'])} - {f(v['max'])})")
    summary['incomplete_runs'] = [r['scenario'] for r in results if not r.get('completed', True)]
    with open(os.path.join(log_dir, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=2)
    print(f'Summary saved: {os.path.join(log_dir, "summary.json")}')


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

    _print_summary(log_dir)
    print('\nAll scenarios have been run')
    print(f'Maps and logs: {log_dir}')


if __name__ == '__main__':
    main()