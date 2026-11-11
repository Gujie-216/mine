"""Paired indoor SLAM experiment. Run from the repository root."""
import argparse
import csv
import json
import math
import platform
import statistics
from pathlib import Path

from demo import RES, ROOM, SCENES, cell, plan, replay


LAG = 10
RADIUS = 0.15
SEEDS = tuple(range(10))
START = (10.8, 4.5)
GOAL = (1.2, 4.5)


def relative(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    c, s = math.cos(a[2]), math.sin(a[2])
    return c*dx + s*dy, -s*dx + c*dy, math.remainder(b[2]-a[2], math.tau)


def trajectory_errors(truth, estimate, lag=LAG):
    if len(truth) != len(estimate) or not 0 < lag < len(truth):
        raise ValueError("Expected synchronized trajectories longer than the positive lag")
    absolute = [math.dist(a[:2], b[:2]) for a, b in zip(truth, estimate)]
    translation, rotation = [], []
    for i in range(len(truth) - lag):
        ref = relative(truth[i], truth[i+lag])
        est = relative(estimate[i], estimate[i+lag])
        translation.append(math.dist(ref[:2], est[:2]))
        rotation.append(math.degrees(math.remainder(ref[2]-est[2], math.tau)))
    return {name: math.sqrt(statistics.mean(v*v for v in values))
            for name, values in [('ate_m', absolute), ('rpe_m', translation),
                                 ('rpe_deg', rotation)]}


def clearance(x, y, obstacles):
    left, bottom, right, top = ROOM
    distances = [x-left, right-x, y-bottom, top-y]
    for left, bottom, right, top in obstacles:
        distances.append(math.hypot(max(left-x, 0, x-right), max(bottom-y, 0, y-top)))
    return min(distances)


def clearance_bound(points, obstacles):
    """Lower bound along every segment using the 1-Lipschitz distance function."""
    minimum = clearance(*points[0], obstacles)
    for (x, y), (u, v) in zip(points, points[1:]):
        length = math.hypot(u-x, v-y)
        n = max(1, math.ceil(length/.02))
        for j in range(n+1):
            distance = clearance(x+(u-x)*j/n, y+(v-y)*j/n, obstacles)
            minimum = min(minimum, distance-length/(2*n))
    return minimum


def wall_distance(x, y, box):
    left, bottom, right, top = box
    if left <= x <= right and bottom <= y <= top:
        return min(x-left, right-x, y-bottom, top-y)
    return math.hypot(max(left-x, 0, x-right), max(bottom-y, 0, y-top))


def trial(scene, seed):
    grid, rows = replay(seed, scene)
    truth = [row[1:4] for row in rows]
    obstacles = SCENES[scene]
    if clearance_bound([p[:2] for p in truth], obstacles) <= RADIUS:
        raise ValueError(f"Invalid acquisition trajectory in {scene}")
    result = dict(scene=scene, seed=seed)
    for label, k in [('odom', 4), ('slam', 7)]:
        errors = trajectory_errors(truth, [row[k:k+3] for row in rows])
        result.update({f'{label}_{key}': value for key, value in errors.items()})
    occupied = [((x+.5)*RES, (y+.5)*RES) for (x,y), v in grid.items() if v > 0]
    walls = (ROOM, *obstacles)
    result['occupied_cells'] = len(occupied)
    result['wall_precision'] = (sum(min(wall_distance(x,y,b) for b in walls) <= RES
                                    for x,y in occupied)/len(occupied) if occupied else 0)
    path = plan(grid, cell(*START), cell(*GOAL))
    centers = [((x+.5)*RES, (y+.5)*RES) for x,y in path]
    bound = clearance_bound(centers, obstacles) if path else None
    result.update(path_found=bool(path), clearance_lower_m=bound,
                  valid_path=bool(path) and bound > RADIUS,
                  path_length_m=(len(path)-1)*RES if path else None)
    return result


def summarize(rows):
    fields = [f'{method}_{metric}' for method in ('odom', 'slam')
              for metric in ('ate_m', 'rpe_m', 'rpe_deg')]
    values = {key: [row[key] for row in rows] for key in [*fields, 'wall_precision']}
    values['paired_ate_gain_m'] = [r['odom_ate_m']-r['slam_ate_m'] for r in rows]
    return dict(trials=len(rows), found=sum(r['path_found'] for r in rows),
                valid=sum(r['valid_path'] for r in rows),
                metrics={key: dict(mean=statistics.mean(v), sample_sd=statistics.stdev(v))
                         for key, v in values.items()})


def run(out):
    records = [trial(scene, seed) for scene in SCENES for seed in SEEDS]
    out.mkdir(parents=True, exist_ok=True)
    with (out/'trials.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=records[0])
        writer.writeheader()
        writer.writerows(records)
    summary = {scene: summarize([r for r in records if r['scene'] == scene]) for scene in SCENES}
    report = dict(protocol=1, python=platform.python_version(), seeds=SEEDS,
                  lag_frames=LAG, robot_radius_m=RADIUS, scenes=summary)
    (out/'summary.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    for scene, row in summary.items():
        print(scene, 'ATE odom / SLAM:', row['metrics']['odom_ate_m']['mean'],
              row['metrics']['slam_ate_m']['mean'], 'valid paths:', row['valid'], '/', row['trials'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=Path('results/benchmark'))
    run(parser.parse_args().out)
