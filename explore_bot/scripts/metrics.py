#!/usr/bin/env python3
"""Score a SLAM-built occupancy grid against the true arena (from arena.sdf).

Typical use from your teammate's iteration script
-------------------------------------------------
    from metrics import evaluate_map, evaluate_live_map, summarize

    # (A) from a saved map (map_saver_cli output):
    r = evaluate_map('runs/run_1.yaml', initial_pose=(x0, y0, yaw0), label='run_1')

    # (B) straight from the running /map topic (needs ROS 2 sourced):
    r = evaluate_live_map(initial_pose=(x0, y0, yaw0), label='run_1')

    print(r['score'])             # 0..100 composite
    results.append(r)
    ...
    print(summarize(results))     # mean / std / min / max over all runs

`initial_pose` is the robot's spawn pose in the WORLD frame (x, y, yaw_rad).
slam_toolbox starts its `map` frame at the robot's starting pose, so every run
with a different start has its map shifted/rotated relative to the arena. The
pose is used to put the map back into the world frame before comparing.
By default a small registration search (+-0.20 m, +-6 deg) removes residual
error from that alignment (e.g. spawn pose off by a few cm); the correction
applied is returned so you can see it. Pass refine=False to disable.

CLI:  python3 metrics.py map.yaml --pose X Y YAW [--no-refine] [--json out.json]

Runner mode (used by launch/demo_launch.py): when SCENARIO_INDEX is set in the
environment and no map file is given on the command line, the settings come from
environment variables instead:
    SCENARIO_INDEX, START_X, START_Y, START_YAW   which run / spawn pose
    EXPLORE_WALL_TIME_S                           mapping time measured by the runner
    MAP_YAML    (optional) saved map, used only if the live /map can't be read
    RESULTS_DIR (optional) where scenario_N_metrics.json is written
    COMPLETED   '1' if explore_lite reported it was done, '0' if the run timed out
"""
import argparse
import json
import math
import os
import sys

import numpy as np
from scipy import ndimage

from ground_truth import build_ground_truth, interior_mask

TOL_M = 0.10  # a map obstacle cell within 10 cm (2 cells) of a true obstacle counts as correct

# Composite-score weights (sum to 1). Documented in README_evaluation.md.
W_COVERAGE, W_F1, W_ACCURACY, W_CLEAN = 0.35, 0.30, 0.20, 0.15


# ----------------------------------------------------------------- map loading
def load_map_yaml(yaml_path):
    """Load a ROS map_server yaml+image. Returns (grid[-1/0/100], res, origin_xy).

    Grid uses OccupancyGrid layout: row 0 = lowest y.
    """
    import yaml
    from PIL import Image

    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img_path = meta['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
    img = np.asarray(Image.open(img_path).convert('L')).astype(float)
    p = img / 255.0 if meta.get('negate', 0) else (255.0 - img) / 255.0
    grid = np.full(img.shape, -1, dtype=np.int8)
    grid[p > meta.get('occupied_thresh', 0.65)] = 100
    grid[p < meta.get('free_thresh', 0.25)] = 0
    # map_saver writes 'unknown' as grey 205, which sits BELOW the usual 0.25
    # free_thresh and would otherwise be read back as free space. Keep it unknown.
    if not meta.get('negate', 0):
        grid[np.abs(img - 205) <= 2] = -1
    grid = grid[::-1]  # image row 0 is top; OccupancyGrid row 0 is bottom
    return grid, float(meta['resolution']), tuple(meta['origin'][:2])


def grid_from_occupancy_msg(msg):
    """nav_msgs/OccupancyGrid -> (grid, res, origin_xy). Map origin yaw assumed 0."""
    grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    o = msg.info.origin.position
    return grid, float(msg.info.resolution), (o.x, o.y)


# ------------------------------------------------------------------- alignment
def _resample_to_world(grid, res, origin, pose, gt_info, gt_shape):
    """Nearest-neighbour resample of a map-frame grid onto the ground-truth world grid."""
    x0, y0, yaw = pose
    n = gt_shape[0]
    gres, gor = gt_info['resolution'], gt_info['origin']
    wx = gor[0] + (np.arange(n) + 0.5) * gres
    wy = gor[1] + (np.arange(n) + 0.5) * gres
    WX, WY = np.meshgrid(wx, wy)
    c, s = math.cos(yaw), math.sin(yaw)
    mx = c * (WX - x0) + s * (WY - y0)
    my = -s * (WX - x0) + c * (WY - y0)
    col = np.floor((mx - origin[0]) / res).astype(int)
    row = np.floor((my - origin[1]) / res).astype(int)
    ok = (row >= 0) & (row < grid.shape[0]) & (col >= 0) & (col < grid.shape[1])
    out = np.full(gt_shape, -1, dtype=np.int8)
    out[ok] = grid[row[ok], col[ok]]
    return out


def _refine_pose(grid, res, origin, pose, gt_dist, gt_info,
                 dxy=0.20, dyaw=math.radians(6)):
    """Small grid search minimising truncated mean distance of map obstacles to truth."""
    rows, cols = np.nonzero(grid == 100)
    if len(rows) < 20:
        return pose
    mx = origin[0] + (cols + 0.5) * res
    my = origin[1] + (rows + 0.5) * res
    gres, gor = gt_info['resolution'], gt_info['origin']
    n = gt_dist.shape[0]

    def cost(p):
        x0, y0, yaw = p
        c, s = math.cos(yaw), math.sin(yaw)
        wx = x0 + c * mx - s * my
        wy = y0 + s * mx + c * my
        ci = np.floor((wx - gor[0]) / gres).astype(int)
        ri = np.floor((wy - gor[1]) / gres).astype(int)
        ok = (ri >= 0) & (ri < n) & (ci >= 0) & (ci < n)
        d = np.full(len(wx), 0.5)
        d[ok] = np.minimum(gt_dist[ri[ok], ci[ok]], 0.5)
        return d.mean()

    best, best_c = pose, cost(pose)
    for step_xy, step_yaw, span_xy, span_yaw in [(0.05, math.radians(2), dxy, dyaw),
                                                  (0.0125, math.radians(0.5), 0.05, math.radians(1.5))]:
        center = best
        nx = int(round(span_xy / step_xy))
        nt = int(round(span_yaw / step_yaw))
        for i in range(-nx, nx + 1):
            for j in range(-nx, nx + 1):
                for k in range(-nt, nt + 1):
                    cand = (center[0] + i * step_xy, center[1] + j * step_xy, center[2] + k * step_yaw)
                    cc = cost(cand)
                    if cc < best_c - 1e-9:
                        best, best_c = cand, cc
    return best


# --------------------------------------------------------------------- metrics
def evaluate_grid(grid, res, origin, initial_pose=(0.0, 0.0, 0.0), refine=True,
                  tol_m=TOL_M, label=None, sdf_path=None):
    """Core evaluation. `grid`: -1 unknown / 0 free / 100 occupied, map-frame."""
    gt, info = build_ground_truth(sdf_path)
    gres = info['resolution']
    tol_cells = tol_m / gres
    gt_occ = gt == 100
    inside = interior_mask(gt, info)
    gt_free_in = (~gt_occ) & inside

    # distance (m) from every cell to the nearest true obstacle cell
    gt_dist = ndimage.distance_transform_edt(~gt_occ) * gres

    pose = tuple(initial_pose)
    if refine:
        pose = _refine_pose(grid, res, origin, pose, gt_dist, info)
    correction = (pose[0] - initial_pose[0], pose[1] - initial_pose[1],
                  math.degrees(pose[2] - initial_pose[2]))

    m = _resample_to_world(grid, res, origin, pose, info, gt.shape)
    m_occ, m_free, m_known = m == 100, m == 0, m != -1
    # ignore map cells outside the arena walls (SLAM sometimes leaks beams through gaps)
    m_occ_in = m_occ & (np.abs(info['origin'][0] + (np.arange(gt.shape[0]) + .5) * gres) <= 4.6)[None, :] \
        & (np.abs(info['origin'][1] + (np.arange(gt.shape[0]) + .5) * gres) <= 4.6)[:, None]

    # --- 1. coverage: how much of the real free interior has been explored
    coverage = (m_free & gt_free_in).sum() / max(1, gt_free_in.sum())

    # --- 2. obstacle precision / recall / F1 with a tolerance band
    map_dist = ndimage.distance_transform_edt(~m_occ_in) * gres  # to nearest map obstacle
    n_map_occ = m_occ_in.sum()
    precision = (m_occ_in & (gt_dist <= tol_m)).sum() / n_map_occ if n_map_occ else 0.0
    # Recall counts only true-obstacle surface cells (outline), not interiors that
    # a lidar can never see.
    gt_edge = gt_occ & ~ndimage.binary_erosion(gt_occ, iterations=1)
    n_edge = gt_edge.sum()
    recall = (gt_edge & (map_dist <= tol_m)).sum() / n_edge if n_edge else 0.0
    f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0

    # --- 3. geometric accuracy: distance from each mapped obstacle to the true one
    if n_map_occ:
        d = gt_dist[m_occ_in]
        mean_err, rmse, p95 = float(d.mean()), float(np.sqrt((d ** 2).mean())), float(np.percentile(d, 95))
    else:
        mean_err = rmse = p95 = float('inf')

    # --- 4. false free space: cells marked free that are really solid obstacle
    #     (cells within the tolerance band of a mapped obstacle are forgiven)
    bad_free = m_free & gt_occ & (map_dist > tol_m)
    n_free_in = (m_free & inside).sum()
    false_free = bad_free.sum() / n_free_in if n_free_in else 0.0

    # --- 5. phantom obstacles: mapped obstacle cells > 2*tol from anything real
    phantom = (m_occ_in & (gt_dist > 2 * tol_m)).sum()
    phantom_frac = phantom / n_map_occ if n_map_occ else 0.0

    # --- 6. overall cell accuracy on cells the robot has classified (tolerance-aware)
    near_edge = ndimage.distance_transform_edt(~gt_edge) * gres <= tol_m
    judged = m_known & inside & ~near_edge
    agree = ((m_occ & gt_occ) | (m_free & ~gt_occ)) & judged
    cell_accuracy = agree.sum() / max(1, judged.sum())

    # --- composite score (0..100)
    acc_term = 0.0 if not np.isfinite(mean_err) else max(0.0, 1 - mean_err / 0.25)
    clean_term = max(0.0, 1 - false_free / 0.05)
    # accuracy/cleanliness only earn credit in proportion to what was actually mapped,
    # so an almost-empty map can't score well by being trivially 'clean'
    score = 100 * (W_COVERAGE * coverage + W_F1 * f1
                   + W_ACCURACY * acc_term * recall + W_CLEAN * clean_term * coverage)

    return {
        'label': label,
        'score': round(float(score), 2),
        'coverage': round(float(coverage), 4),
        'obstacle_precision': round(float(precision), 4),
        'obstacle_recall': round(float(recall), 4),
        'obstacle_f1': round(float(f1), 4),
        'mean_error_m': round(mean_err, 4),
        'rmse_m': round(rmse, 4),
        'p95_error_m': round(p95, 4),
        'false_free_fraction': round(float(false_free), 4),
        'phantom_obstacle_fraction': round(float(phantom_frac), 4),
        'cell_accuracy': round(float(cell_accuracy), 4),
        'alignment_correction': {'dx_m': round(correction[0], 3), 'dy_m': round(correction[1], 3),
                                 'dyaw_deg': round(correction[2], 2)},
        'initial_pose': list(initial_pose),
    }


def evaluate_map(map_yaml, initial_pose=(0.0, 0.0, 0.0), refine=True, label=None, **kw):
    """Evaluate a saved map (yaml+pgm/png) against the arena."""
    grid, res, origin = load_map_yaml(map_yaml)
    return evaluate_grid(grid, res, origin, initial_pose, refine, label=label or os.path.basename(map_yaml), **kw)


def evaluate_live_map(initial_pose=(0.0, 0.0, 0.0), refine=True, label=None, topic='/map', timeout=15.0, **kw):
    """Grab one /map message from the running SLAM node and evaluate it."""
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

    owns_rclpy = not rclpy.ok()
    if owns_rclpy:
        rclpy.init()
    node = rclpy.create_node('map_evaluator')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    box = {}
    node.create_subscription(OccupancyGrid, topic, lambda m: box.setdefault('msg', m), qos)
    end = node.get_clock().now().nanoseconds + int(timeout * 1e9)
    while 'msg' not in box and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    if owns_rclpy:
        rclpy.shutdown()
    if 'msg' not in box:
        raise TimeoutError(f'no message on {topic} within {timeout}s')
    grid, res, origin = grid_from_occupancy_msg(box['msg'])
    return evaluate_grid(grid, res, origin, initial_pose, refine, label=label, **kw)


def summarize(results):
    """Aggregate a list of evaluate_* result dicts (e.g. the 5 runs)."""
    keys = ['score', 'coverage', 'obstacle_precision', 'obstacle_recall', 'obstacle_f1',
            'mean_error_m', 'rmse_m', 'p95_error_m', 'false_free_fraction',
            'phantom_obstacle_fraction', 'cell_accuracy']
    out = {'n_runs': len(results)}
    for k in keys:
        v = np.array([r[k] for r in results], dtype=float)
        v = v[np.isfinite(v)]
        out[k] = {'mean': round(float(v.mean()), 4), 'std': round(float(v.std()), 4),
                  'min': round(float(v.min()), 4), 'max': round(float(v.max()), 4)} if len(v) else None
    return out


def _print_report(r):
    """Human-readable block for the terminal: coverage, F1, mean error, false-free, score, times."""
    sim, wall = r.get('mapping_sim_time_s'), r.get('mapping_wall_time_s')
    print(f"  --- Scenario {r.get('scenario', '?')} results"
          f"{'' if r.get('completed', True) else ' (INCOMPLETE: timed out)'} ---")
    print(f"  Mapping time (sim):  {f'{sim:.1f} s' if sim is not None else 'unavailable'}")
    print(f"  Mapping time (real): {f'{wall:.1f} s' if wall is not None else 'unavailable'}")
    print(f"  Coverage:            {r['coverage'] * 100:.1f} %")
    print(f"  Obstacle F1:         {r['obstacle_f1']:.3f}")
    print(f"  Mean error:          {r['mean_error_m'] * 100:.1f} cm")
    print(f"  False-free:          {r['false_free_fraction'] * 100:.1f} %")
    print(f"  Composite score:     {r['score']:.1f} / 100")


def run_from_env():
    """Runner mode: evaluate one scenario from environment variables, save + print."""
    n = int(os.environ['SCENARIO_INDEX'])
    pose = (float(os.environ.get('START_X', 0)), float(os.environ.get('START_Y', 0)),
            float(os.environ.get('START_YAW', 0)))
    map_yaml = os.environ.get('MAP_YAML')
    try:  # preferred: the live map keeps exact unknown/free/occupied values
        r = evaluate_live_map(pose, label=f'scenario_{n}')
        r['map_source'] = 'live /map'
    except Exception as e:
        if not (map_yaml and os.path.exists(map_yaml)):
            raise
        print(f'  Live /map unavailable ({e}); scoring the saved map instead.')
        r = evaluate_map(map_yaml, pose, label=f'scenario_{n}')
        r['map_source'] = map_yaml
    r['scenario'] = n
    sim = os.environ.get('EXPLORE_SIM_TIME_S')
    wall = os.environ.get('EXPLORE_WALL_TIME_S')
    r['mapping_sim_time_s'] = float(sim) if sim else None   # Gazebo /clock
    r['mapping_wall_time_s'] = float(wall) if wall else None  # real time
    r['completed'] = os.environ.get('COMPLETED', '1') == '1'
    out_dir = os.environ.get('RESULTS_DIR')
    if out_dir:
        path = os.path.join(out_dir, f'scenario_{n}_metrics.json')
        with open(path, 'w') as f:
            json.dump(r, f, indent=2)
    _print_report(r)
    if out_dir:
        print(f'  Results saved: {path}')


def main():
    if 'SCENARIO_INDEX' in os.environ and len(sys.argv) == 1:
        return run_from_env()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('map_yaml')
    ap.add_argument('--pose', nargs=3, type=float, default=[0, 0, 0], metavar=('X', 'Y', 'YAW_RAD'),
                    help='robot spawn pose in the world frame')
    ap.add_argument('--no-refine', action='store_true')
    ap.add_argument('--json')
    a = ap.parse_args()
    r = evaluate_map(a.map_yaml, tuple(a.pose), refine=not a.no_refine)
    print(json.dumps(r, indent=2))
    if a.json:
        with open(a.json, 'w') as f:
            json.dump(r, f, indent=2)


if __name__ == '__main__':
    main()