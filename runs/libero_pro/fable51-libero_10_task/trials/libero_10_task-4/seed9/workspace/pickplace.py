#!/usr/bin/env python3
"""Rim-grasp pick and place of a mug.
Usage: pickplace.py <mug_cx> <mug_cy> <rim_top_z> <plate_cx> <plate_cy> [--dry]
Grasp: hand down, fingers along world x, TCP on the -x rim wall.
"""
import sys
import numpy as np, rclpy
from ctl import Ctl, DOWN_X

WALL_R = 0.044          # mid-wall radius of the mug rim
GRASP_DEPTH = 0.025     # fingertips this far below the rim top
SAFE_Z = 0.78
PLACE_TCP_Z = 0.545     # rim top when mug sits on plate ~0.56 -> tcp 0.535; release 1 cm above


def main():
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    cx, cy, rim, px, py = map(float, a)
    dry = "--dry" in sys.argv
    c = Ctl()
    q = DOWN_X
    gx = cx - WALL_R          # TCP x so that the wall sits between the fingers
    grasp_z = rim - GRASP_DEPTH
    wps = [
        ("pre-grasp", (gx, cy, SAFE_Z)),
        ("descend-1", (gx, cy, rim + 0.03)),
        ("grasp", (gx, cy, grasp_z)),
        ("lift", (gx, cy, SAFE_Z)),
        ("mid", ((gx + px - WALL_R) / 2, (cy + py) / 2, SAFE_Z + 0.03)),
        ("over-plate", (px - WALL_R, py, SAFE_Z)),
        ("lower-1", (px - WALL_R, py, PLACE_TCP_Z + 0.05)),
        ("place", (px - WALL_R, py, PLACE_TCP_Z)),
        ("retreat", (px - WALL_R, py, SAFE_Z)),
    ]
    # pre-solve IK chain
    seed = c.joints()[0]
    sols = {}
    for name, p in wps:
        s = c.ik_tcp(p, q, seed)
        if s is None:
            print("IK FAIL at", name, p); sys.exit(2)
        d = np.abs(np.array(s) - np.array(seed)).max() if seed is not None else 0
        print(f"IK {name:10s} {np.round(p,3)} -> {np.round(s,3).tolist()} (dmax {d:.2f})")
        sols[name] = s
        seed = s
    if dry:
        rclpy.shutdown(); return

    def go(name, secs=3.0):
        print(f"== {name}")
        ok = c.move_joints(sols[name], secs)
        pos, qt, tcp = c.fk_hand()
        print(f"   tcp {tcp.round(4)} target {np.round(dict(wps)[name],4)} ok={ok}")
        return ok

    f = c.joints()[1]
    if f < 0.035:
        c.gripper(True)
    go("pre-grasp", 4.0)
    go("descend-1", 2.5)
    go("grasp", 2.0)
    w0 = c.wrench()
    gap = c.gripper(False)
    print(f"   finger after close: {gap:.4f}  wrench {c.wrench().round(2)} (before {w0.round(2)})")
    if gap < 0.002:
        print("GRASP FAILED (closed on air); reopening and stopping")
        c.gripper(True); go("lift", 2.5); sys.exit(3)
    go("lift", 3.0)
    print(f"   holding: finger={c.joints()[1]:.4f} wrench {c.wrench().round(2)}")
    go("mid", 3.0)
    go("over-plate", 3.0)
    go("lower-1", 2.5)
    go("place", 2.0)
    c.gripper(True)
    go("retreat", 3.0)
    print("DONE")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
