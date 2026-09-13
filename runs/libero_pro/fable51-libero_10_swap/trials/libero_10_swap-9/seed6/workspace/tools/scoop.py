#!/usr/bin/env python3
"""Right the lying mug by 'scooping': closed fingers into the mouth just below
the axis height, then move the TCP along an arc about the base edge on the table
so the rim end lifts and the mug tips onto its base.
  python3 tools/scoop.py info|run [pitch_deg]
Mug lying along y, rim at -y end (Y_R), base end at Y_B, axis at (AX_X, AX_Z)."""
import sys
import numpy as np
import rclpy
sys.path.insert(0, "/workspace/tools")
from robot import mat_to_quat
from planner import Planner

AX_X, AX_Z = -0.165, 0.953
Y_R, Y_B = -0.52, -0.415
PITCH = np.deg2rad(float(sys.argv[2]) if len(sys.argv) > 2 else 60.0)
DEPTH = min(0.02, 0.0345 / np.tan(PITCH))
C = np.array([Y_B, 0.90])                       # pivot (y, z)


def frame():
    a = np.array([0.0, np.cos(PITCH), -np.sin(PITCH)])
    yh = np.array([1.0, 0.0, 0.0])              # closing axis along x (irrelevant, fingers closed)
    xh = np.cross(yh, a)
    R = np.stack([xh, yh, a], 1)
    return a, R, mat_to_quat(R)


def arc_points(n):
    tcp0 = np.array([AX_X, Y_R + DEPTH, AX_Z - 0.01])
    v0 = np.array([tcp0[1] - C[0], tcp0[2] - C[1]])
    ang0, rad = np.arctan2(v0[1], v0[0]), np.linalg.norm(v0)
    pts = []
    for th in np.linspace(0, np.pi / 2, n + 1)[1:]:
        ang = ang0 - th
        pts.append(np.array([AX_X, C[0] + rad * np.cos(ang), C[1] + rad * np.sin(ang)]))
    return tcp0, pts


def main():
    stage = sys.argv[1]
    rclpy.init(); p = Planner()
    a, R, quat = frame()
    tcp0, pts = arc_points(6)
    print("tcp0", np.round(tcp0, 4), "depth", round(DEPTH, 4))
    if stage == "info":
        seed = None
        for q_ in [tcp0 - 0.04 * a, tcp0] + pts:
            s = p.ik(q_, quat, seed=seed, attempts=4)
            print(np.round(q_, 3), "ok" if s is not None else "FAIL")
            if s is not None:
                seed = s
    elif stage == "run":
        p.set_scene(microwave="solid", yellow=False)
        # clear the mug, close fingers
        tcp, _, Rc = p.tcp()
        p.line(tcp + np.array([0, 0, 0.05]), mat_to_quat(Rc), step=0.01, avoid=False, min_fraction=0.5)
        p.grip(False)
        pre = tcp0 - 0.04 * a
        ok = p.goto_tcp(pre, quat, time_s=8.0)
        print("pre ok", ok, "tcp", np.round(p.tcp()[0], 4))
        ok = p.line(tcp0, quat, step=0.005, avoid=False, time_scale=4)
        print("insert ok", ok, "tcp", np.round(p.tcp()[0], 4))
        for i, pt in enumerate(pts):
            ok = p.line(pt, quat, step=0.005, avoid=False, min_fraction=0.9, time_scale=5)
            print(f"arc {i} target {np.round(pt, 3)} ok {ok} tcp {np.round(p.tcp()[0], 4)}", flush=True)
            if not ok:
                break
        # retreat: up, then back along -y
        tcp = p.tcp()[0]
        ok = p.line(tcp + np.array([0, 0, 0.08]), quat, step=0.01, avoid=False, min_fraction=0.9, time_scale=3)
        print("retreat ok", ok, "tcp", np.round(p.tcp()[0], 4))
    p.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
