#!/usr/bin/env python3
"""Rim-grasp pick and place for one mug.

Usage: python3 pick_place.py <grasp_x> <grasp_y> <grasp_z> <place_x> <place_y> <place_z>
All in world frame; grasp = TCP position on the mug rim wall, place = TCP
position at release. Fingers open along world y (hand yaw 0).
"""
import sys

import numpy as np

import rob

Z_TRAVEL = 0.75
R = rob.top_down_R(0.0)


def move_tcp(r, p, seed, seconds, tag):
    q = r.ik_tcp(p, R, seed=seed)
    if q is None:
        raise SystemExit(f"{tag}: IK failed for {p}")
    for attempt in range(6):
        code, qf = r.move_traj([q], [seconds])
        if np.abs(qf - q).max() < 0.02:
            break
        print(f"{tag}: residual too big, resending")
    else:
        raise SystemExit(f"{tag}: could not converge")
    tcp = r.tcp_world(qf)
    print(f"{tag}: tcp world {tcp.round(3)} (target {np.round(p, 3)})")
    return qf


def main():
    gx, gy, gz, px, py, pz = map(float, sys.argv[1:7])
    r = rob.Robot("pick_place")
    q = r.q()
    print("start q", q.round(3), "tcp", r.tcp_world(q).round(3))

    r.open()
    q = move_tcp(r, (gx, gy, Z_TRAVEL), q, 6.0, "pre-grasp")
    q = move_tcp(r, (gx, gy, gz + 0.06), q, 2.5, "approach")
    q = move_tcp(r, (gx, gy, gz), q, 2.0, "grasp-height")
    f = r.close()
    gap = f[0] - f[1]
    print(f"finger gap after close: {gap:.4f}")
    if gap < 0.004:
        raise SystemExit("closed on air; aborting")
    q = move_tcp(r, (gx, gy, Z_TRAVEL), q, 3.0, "lift")
    f = r.fingers()
    print(f"finger gap after lift: {f[0]-f[1]:.4f}")
    q = move_tcp(r, (px, py, Z_TRAVEL), q, 6.0, "transport")
    q = move_tcp(r, (px, py, pz + 0.05), q, 2.5, "pre-place")
    q = move_tcp(r, (px, py, pz), q, 2.0, "place")
    r.open()
    q = move_tcp(r, (px, py, Z_TRAVEL), q, 3.0, "retreat")
    print("done")


if __name__ == "__main__":
    main()
