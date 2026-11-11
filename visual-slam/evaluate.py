"""Evaluate one full-frame monocular replay. Positions in metres, time in seconds."""
import argparse
import json
from pathlib import Path

import numpy as np


def rotations(xyzw):
    norms = np.linalg.norm(xyzw, axis=1)
    if np.any(norms < 1e-8):
        raise ValueError("Zero quaternion in trajectory")
    x, y, z, w = (xyzw / norms[:, None]).T
    return np.stack([1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w),
                     2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w),
                     2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)], axis=1).reshape(-1, 3, 3)


def reference(root, dataset):
    if dataset == "tum":
        a = np.loadtxt(root / "groundtruth.txt")
        return a[:, 0], a[:, 1:4], rotations(a[:, 4:8])
    import yaml
    a = np.loadtxt(root / "mav0/state_groundtruth_estimate0/data.csv", delimiter=",")
    with (root / "mav0/cam0/sensor.yaml").open() as f:
        body_from_camera = np.array(yaml.safe_load(f)["T_BS"]["data"]).reshape(4, 4)
    body = rotations(a[:, [5, 6, 7, 4]])
    return (a[:, 0]*1e-9, a[:, 1:4] + body @ body_from_camera[:3, 3],
            body @ body_from_camera[:3, :3])


def align(x, y):
    xc, yc = x-x.mean(0), y-y.mean(0)
    u, singular, vt = np.linalg.svd(yc.T @ xc)
    if singular[1] <= 1e-12:
        raise ValueError("Trajectory has insufficient geometry for Sim(3) alignment")
    sign = np.array([1., 1., np.linalg.det(u @ vt)])
    r = (u*sign) @ vt
    scale = singular.dot(sign)/np.sum(xc*xc)
    return scale*(xc @ r.T) + y.mean(0), r, scale


def nearest(times, query):
    right = np.clip(np.searchsorted(times, query), 0, len(times)-1)
    left = np.maximum(0, right-1)
    return np.where(abs(times[left]-query) < abs(times[right]-query), left, right)


def evaluate(frames, gt, tolerance):
    t, xyz, rot = gt
    stamp = frames["stamp"]
    if len(frames) < 2 or np.any(np.diff(stamp) <= 0) or np.any(np.diff(t) <= 0):
        raise ValueError("Expected strictly increasing timestamps and at least two frames")
    wall = frames["wall_s"][-1] - frames["wall_s"][0]
    cpu = frames["cpu_s"][-1] - frames["cpu_s"][0]
    if wall <= 0 or cpu < 0:
        raise ValueError("Invalid process measurement interval")
    valid = frames["valid"] == 1
    maps = np.unique(frames["map"][valid])
    result = dict(frames=len(frames), coverage=float(valid.mean()), valid_maps=len(maps),
                  mean_frame_ms=float(frames["frame_ms"].mean()),
                  p95_frame_ms=float(np.percentile(frames["frame_ms"], 95)),
                  process_cpu_pct=float(100*cpu/wall), cpu_ms_per_frame=float(1000*cpu/(len(frames)-1)),
                  ate_m=None, rpe_1s_m=None, rpe_1s_deg=None)
    idx = nearest(t, stamp)
    keep = valid & (abs(t[idx]-stamp) <= tolerance)
    result["associated"] = int(keep.sum())
    result["complete"] = bool(len(maps) == 1 and keep.mean() >= .95)
    if len(maps) != 1 or keep.sum() < 3:
        return result
    f = frames[keep]
    x = np.column_stack([f[k] for k in ("x", "y", "z")])
    y = xyz[idx[keep]]
    try:
        x, r, scale = align(x, y)
    except ValueError:
        result["complete"] = False
        return result
    result["scale"] = float(scale)
    result["ate_m"] = float(np.sqrt(np.mean(np.sum((x-y)**2, axis=1))))
    est = r @ rotations(np.column_stack([f[k] for k in ("qx", "qy", "qz", "qw")]))
    ref = rot[idx[keep]]
    a = np.arange(len(f))
    b = nearest(f["stamp"], f["stamp"] + 1.)
    # Reject intervals spanning a lost frame or an unassociated reference sample.
    gaps = np.cumsum(~keep)[keep]
    ok = (abs(f["stamp"][b]-f["stamp"]-1.) <= .03) & (gaps[b] == gaps[a])
    a, b = a[ok], b[ok]
    result["rpe_pairs"] = len(a)
    if len(a):
        def relative(p, orientations):
            inv = orientations[a].transpose(0, 2, 1)
            return (inv @ (p[b]-p[a])[:, :, None])[:, :, 0], inv @ orientations[b]
        dp, dr = relative(x, est)
        gp, gr = relative(y, ref)
        cos_angle = (np.sum(dr*gr, axis=(1, 2))-1)/2
        angles = np.degrees(np.arccos(np.clip(cos_angle, -1, 1)))
        result["rpe_1s_m"] = float(np.sqrt(np.mean(np.sum((dp-gp)**2, axis=1))))
        result["rpe_1s_deg"] = float(np.sqrt(np.mean(angles**2)))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", choices=["tum", "euroc"])
    parser.add_argument("sequence", type=Path)
    parser.add_argument("log", type=Path)
    args = parser.parse_args()
    frames = np.atleast_1d(np.genfromtxt(args.log, names=True, delimiter=","))
    print(json.dumps(evaluate(frames, reference(args.sequence, args.dataset),
                              .005 if args.dataset == "euroc" else .02), indent=2))
