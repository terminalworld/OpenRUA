#!/usr/bin/env python3
"""Closed-loop insertion of the held bowl into the open bottom drawer.

Each step: grab wrist-camera depth+color -> measure bowl rim (yellow px,
fixed-radius circle fit), handle +y edge and drawer front wall inner face
in the SAME frame -> correct y -> descend one step. Stops on stall.
"""
import subprocess
import sys

import cv2
import numpy as np
from scipy.optimize import least_squares

import ctl

R_RIM = 0.0551
H_BOWL = 0.052
Q = ctl.Q_DOWN_FINGERS_X


def measure():
    subprocess.run([sys.executable, "scene.py", "robot0_eye_in_hand"],
                   check=True, capture_output=True)
    img = cv2.imread("snaps/robot0_eye_in_hand.png")
    xyz = np.load("snaps/robot0_eye_in_hand_xyz.npy")
    z = xyz[..., 2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = (hsv[..., 0] > 18) & (hsv[..., 0] < 40) & (hsv[..., 1] > 80) & (hsv[..., 2] > 120)
    ys, xs = np.nonzero(m)
    P = xyz[ys, xs]
    ok = np.isfinite(P[:, 2]) & (P[:, 2] > 0.93)
    q = P[ok][:, :2]
    res = least_squares(lambda c: np.hypot(q[:, 0] - c[0], q[:, 1] - c[1]) - R_RIM,
                        x0=[q[:, 0].mean(), q[:, 1].mean()])
    out = {"bowl": (res.x[0], res.x[1]), "rim_z": float(np.median(P[ok][:, 2])),
           "rms": float(np.sqrt(np.mean(res.fun ** 2))), "n": int(ok.sum())}
    # middle-drawer handle: protrusion at z 1.0-1.04, x in bar range
    k = (z > 1.0) & (z < 1.04) & (xyz[..., 0] > -0.16) & (xyz[..., 0] < -0.05) & \
        (xyz[..., 1] < -0.15) & (xyz[..., 1] > -0.23)
    out["handle_y"] = float(xyz[k][:, 1].max()) if k.sum() > 5 else None
    # drawer front wall top: z ~0.983, take its inner (min y) face
    k = (z > 0.975) & (z < 0.992) & (xyz[..., 0] > -0.21) & (xyz[..., 0] < -0.02) & \
        (xyz[..., 1] > -0.12) & (xyz[..., 1] < -0.04)
    out["wall_in_y"] = float(np.percentile(xyz[k][:, 1], 2)) if k.sum() > 5 else None
    return out


def report(m, r):
    tcp = r.fk_tcp()[0]
    bx, by = m["bowl"]
    zb = m["rim_z"] - H_BOWL
    s = (f"tcp=({tcp[0]:.4f},{tcp[1]:.4f},{tcp[2]:.4f}) bowl=({bx:.4f},{by:.4f}) "
         f"rim_z={m['rim_z']:.4f} bottom={zb:.4f} rms={m['rms']*1000:.1f}mm n={m['n']}")
    if m["handle_y"] is not None:
        s += f" | handle_edge={m['handle_y']:.4f} margin={((by - R_RIM) - m['handle_y'])*1000:.1f}mm"
    if m["wall_in_y"] is not None:
        s += f" | wall_in={m['wall_in_y']:.4f} margin={(m['wall_in_y'] - (by + R_RIM))*1000:.1f}mm"
    print(s, flush=True)
    return tcp, (bx, by), zb


if __name__ == "__main__":
    r = ctl.Robot("insert")
    m = measure()
    report(m, r)


def measure_agent():
    """Bowl rim + middle-handle edge from the static agentview camera."""
    subprocess.run([sys.executable, "scene.py", "agentview"],
                   check=True, capture_output=True)
    img = cv2.imread("snaps/agentview.png")
    xyz = np.load("snaps/agentview_xyz.npy")
    z = xyz[..., 2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = (hsv[..., 0] > 18) & (hsv[..., 0] < 40) & (hsv[..., 1] > 80) & (hsv[..., 2] > 120)
    ys, xs = np.nonzero(m)
    P = xyz[ys, xs]
    ok = np.isfinite(P[:, 2]) & (P[:, 2] > 0.93) & (P[:, 0] > -0.3) & (P[:, 0] < 0.0)
    q = P[ok][:, :2]
    res = least_squares(lambda c: np.hypot(q[:, 0] - c[0], q[:, 1] - c[1]) - R_RIM,
                        x0=[q[:, 0].mean(), q[:, 1].mean()])
    k = (z > 1.0) & (z < 1.04) & (xyz[..., 0] > -0.16) & (xyz[..., 0] < -0.05) & \
        (xyz[..., 1] < -0.15) & (xyz[..., 1] > -0.23)
    hy = float(xyz[k][:, 1].max()) if k.sum() > 5 else None
    by = res.x[1]
    print(f"agentview: bowl=({res.x[0]:.4f},{by:.4f}) rim_z={np.median(P[ok][:,2]):.4f} "
          f"rms={np.sqrt(np.mean(res.fun**2))*1000:.1f}mm n={ok.sum()} handle_edge={hy} "
          f"margin={((by - R_RIM) - hy)*1000 if hy else float('nan'):.1f}mm", flush=True)
    return {"bowl": (res.x[0], by), "handle_y": hy}
