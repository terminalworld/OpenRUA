#!/usr/bin/env python3
"""Pick alphabet soup and butter, place both in the basket."""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from robot import Robot
import scene

# desired panda_hand orientation: z down, fingers along world y
HAND_DOWN = np.array([1.0, 0.0, 0.0, 0.0])
# IK target link is yawed 45deg from panda_hand: pre-rotate the request
IK_Q = (Rot.from_quat(HAND_DOWN) * Rot.from_euler("z", -45, degrees=True)).as_quat()

TABLE_Z = 0.425
BASKET = np.array([0.005, 0.25])
BASKET_DROP_Z = 0.72

OBJECTS = {
    "soup":   dict(xy=np.array([-0.078, -0.174]), top=0.50, grasp_z=0.475, lid=(0.488, 0.515)),
    "butter": dict(xy=np.array([0.047, 0.067]),  top=0.449, grasp_z=0.442, lid=(0.443, 0.46)),
}


def refine(r, xy, lid):
    """Look straight down from the eye-in-hand camera; centroid of the top face."""
    img, d, pw = scene.cloud("robot0_eye_in_hand")
    z = pw[:, :, 2]
    m = (z > lid[0]) & (z < lid[1]) & (np.abs(pw[:, :, 0] - xy[0]) < 0.06) & (np.abs(pw[:, :, 1] - xy[1]) < 0.06)
    p = pw[m]
    if len(p) < 50:
        print(f"  refine: only {len(p)} top points, keeping {xy}", flush=True)
        return xy
    c = p[:, :2].mean(0)
    print(f"  refine: n={len(p)} top-face centroid {np.round(c,4)} (prior {xy}) "
          f"x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}]", flush=True)
    if np.linalg.norm(c - xy) > 0.03:
        print("  refine: shift >3cm, suspicious; averaging", flush=True)
        c = (c + xy) / 2
    return c


def pick_place(r, name):
    o = OBJECTS[name]
    xy = o["xy"]
    print(f"== {name}: open gripper", flush=True)
    r.gripper(0.04)
    print(f"== {name}: pre-grasp above", flush=True)
    r.move_tcp([xy[0], xy[1], 0.60], IK_Q, 4.0)
    xy = refine(r, xy, o["lid"])
    r.move_tcp([xy[0], xy[1], 0.60], IK_Q, 1.5)
    print(f"== {name}: descend", flush=True)
    r.move_tcp([xy[0], xy[1], o["top"] + 0.02], IK_Q, 2.0)
    r.move_tcp([xy[0], xy[1], o["grasp_z"]], IK_Q, 2.0)
    print(f"== {name}: close", flush=True)
    f = r.gripper(0.0)
    gap = abs(f[0]) + abs(f[1])
    print(f"  finger gap {gap:.4f}", flush=True)
    if gap < 0.008:
        print("  GRASP FAILED (closed on air)", flush=True)
        return False
    print(f"== {name}: lift", flush=True)
    r.move_tcp([xy[0], xy[1], 0.72], IK_Q, 3.0)
    f = r.fingers(); print(f"  fingers after lift {f}", flush=True)
    print(f"== {name}: over basket", flush=True)
    r.move_tcp([BASKET[0], BASKET[1], BASKET_DROP_Z], IK_Q, 4.0)
    f = r.fingers(); print(f"  fingers over basket {f}", flush=True)
    print(f"== {name}: release", flush=True)
    r.gripper(0.04)
    r.move_tcp([BASKET[0], BASKET[1], 0.78], IK_Q, 1.5)
    return True


if __name__ == "__main__":
    names = sys.argv[1:] or ["soup", "butter"]
    r = Robot()
    for n in names:
        ok = pick_place(r, n)
        print(f"#### {n}: {'OK' if ok else 'FAIL'}", flush=True)
        if not ok:
            break
    print("#### DONE", flush=True)
