"""Indoor lidar SLAM and navigation experiment. Python 3.9+, standard library."""
import argparse
import csv
import heapq
import json
import math
import random
from collections import defaultdict
from pathlib import Path

RES = 0.15
LIMIT = 10.0
BEAMS = 90
ANGLES = [i * math.tau / BEAMS for i in range(BEAMS)]


def cell(x, y):
    return math.floor(x / RES), math.floor(y / RES)


ROOM = (0, 0, 12, 9)
SCENES = {
    "open": (),
    "furnished": ((4.5, 3.7, 7, 5.5), (8, 5.9, 9, 6.4)),
    "partitioned": ((3.8, 3.0, 4.2, 5.8), (6.0, 2.7, 8.6, 3.1),
                    (6.0, 5.8, 8.5, 6.2)),
}


def simulate(seed, scene="furnished"):
    """Generate local-frame odometry, ranges and a separate evaluation pose."""
    rng = random.Random(seed)
    boxes = [ROOM, *SCENES[scene]]
    walls = []
    for left, bottom, right, top in boxes:
        walls.extend([(0, left, bottom, top), (0, right, bottom, top),
                      (1, bottom, left, right), (1, top, left, right)])
    last = None
    for i in range(161):
        t = math.tau * i / 160
        pose = (6 + 4.8 * math.cos(t), 4.5 + 3.4 * math.sin(t),
                math.atan2(3.4 * math.cos(t), -4.8 * math.sin(t)))
        x, y, yaw = pose
        ranges = []
        for angle in ANGLES:
            direction = (math.cos(yaw + angle), math.sin(yaw + angle))
            hit = LIMIT
            for axis, value, low, high in walls:
                if abs(direction[axis]) < 1e-8:
                    continue
                distance = (value - pose[axis]) / direction[axis]
                cross = pose[1-axis] + distance * direction[1-axis]
                if 0 < distance < hit and low <= cross <= high:
                    hit = distance
            ranges.append(min(LIMIT, hit + rng.gauss(0, .015)) if hit < LIMIT else LIMIT)
        motion = (0, 0, 0)
        if last:
            dx, dy = x - last[0], y - last[1]
            c, s = math.cos(last[2]), math.sin(last[2])
            rotation = math.remainder(yaw - last[2], math.tau)
            motion = (1.035 * (c*dx + s*dy) + rng.gauss(0, .008),
                      -s*dx + c*dy + rng.gauss(0, .005),
                      rotation + .002 + rng.gauss(0, .002))
        yield motion, ranges, pose
        last = pose


def advance(pose, motion):
    x, y, yaw = pose
    dx, dy, turn = motion
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c*dx - s*dy, y + s*dx + c*dy, yaw + turn


def match(grid, predicted, ranges):
    x, y, yaw = predicted
    best, result = -math.inf, predicted
    for turn in (0, -.016, .016, -.032, .032):
        heading = yaw + turn
        points = [(x + r*math.cos(heading+a), y + r*math.sin(heading+a))
                  for a, r in zip(ANGLES[::2], ranges[::2]) if r < LIMIT]
        for dx in (0, -.06, .06):
            for dy in (0, -.06, .06):
                score = sum(max(0, grid.get(cell(px+dx, py+dy), 0))
                            for px, py in points)
                score -= 30*(abs(dx) + abs(dy)) + 80*abs(turn)
                if score > best:
                    best, result = score, (x+dx, y+dy, heading)
    return result


def integrate(grid, pose, ranges):
    x, y, yaw = pose
    free, occupied = set(), set()
    for angle, distance in zip(ANGLES, ranges):
        c, s = math.cos(yaw+angle), math.sin(yaw+angle)
        steps = math.ceil(distance / (RES*.5))
        for j in range(steps):
            d = distance*j/steps
            free.add(cell(x+d*c, y+d*s))
        if distance < LIMIT:
            occupied.add(cell(x+distance*c, y+distance*s))
    for p in free - occupied:
        grid[p] = max(-8, grid[p]-1)
    for p in occupied:
        grid[p] = min(8, grid[p]+3)


def plan(grid, start, goal):
    # Keep a one-cell margin around obstacles and unexplored space.
    safe = {p for p in grid if all(grid.get((p[0]+i,p[1]+j),0) < 0
            for i in (-1,0,1) for j in (-1,0,1))}
    if start not in safe or goal not in safe:
        return []
    queue, cost, parent = [(0, start)], {start: 0}, {start: None}
    while queue:
        _, p = heapq.heappop(queue)
        if p == goal:
            path = []
            while p is not None:
                path.append(p)
                p = parent[p]
            return path[::-1]
        for dx, dy in ((0,1),(0,-1),(1,0),(-1,0)):
            q = p[0]+dx, p[1]+dy
            g = cost[p]+1
            if q in safe and g < cost.get(q, math.inf):
                cost[q], parent[q] = g, p
                heapq.heappush(queue, (g+abs(q[0]-goal[0])+abs(q[1]-goal[1]), q))
    return []


def replay(seed, scene):
    grid, rows = defaultdict(int), []
    for i, (motion, ranges, truth) in enumerate(simulate(seed, scene)):
        if i == 0:
            # Common initial reference for evaluation.
            odom = estimate = truth
        else:
            odom = advance(odom, motion)
            estimate = match(grid, advance(estimate, motion), ranges)
        integrate(grid, estimate, ranges)
        rows.append((i, *truth, *odom, *estimate))
    return grid, rows


def run(seed, out, scene="furnished"):
    out.mkdir(parents=True, exist_ok=True)
    grid, rows = replay(seed, scene)
    with (out/'trajectory.csv').open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['frame']+[f'{kind}_{axis}' for kind in ('truth','odom','slam')
                                    for axis in ('x','y','yaw')])
        writer.writerows(rows)
    def rmse(k):
        return math.sqrt(sum((r[k]-r[1])**2+(r[k+1]-r[2])**2 for r in rows)/len(rows))
    path = plan(grid, cell(*rows[0][7:9]), cell(*rows[80][7:9]))
    metrics = dict(scene=scene, seed=seed, frames=len(rows), odom_rmse_m=rmse(4), slam_rmse_m=rmse(7),
                   path_found=bool(path), path_length_m=(len(path)-1)*RES if path else None)
    (out/'metrics.json').write_text(json.dumps(metrics, indent=2), encoding='utf-8')
    with (out/'map.svg').open('w', encoding='utf-8') as f:
        xs, ys = zip(*grid)
        f.write(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{min(xs)} {-max(ys)-1} {max(xs)-min(xs)+1} {max(ys)-min(ys)+1}">')
        f.write('<rect x="-100%" y="-100%" width="300%" height="300%" fill="#c4c8cc"/>')
        for (x,y), value in grid.items():
            color = '#263544' if value > 0 else '#fff' if value < 0 else '#c4c8cc'
            f.write(f'<rect x="{x}" y="{-y-1}" width="1" height="1" fill="{color}"/>')
        for points, color in [([(r[7]/RES,-r[8]/RES) for r in rows], '#2672c9'),
                              ([(x+.5,-y-.5) for x,y in path], '#ed671d')]:
            coords = ' '.join(f'{x:.2f},{y:.2f}' for x,y in points)
            f.write(f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width=".55"/>')
        f.write('</svg>')
    print(json.dumps(metrics, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--scene', choices=SCENES, default="furnished")
    parser.add_argument('--out', type=Path, default=Path('results'))
    args = parser.parse_args()
    run(args.seed, args.out, args.scene)
