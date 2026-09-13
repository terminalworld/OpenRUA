#!/usr/bin/env python3
"""Continuation IK probe: march the TCP outward along -y from a feasible pose,
seeding each step with the previous solution. Reports the last feasible y.
usage: reach_probe.py yaw_deg pitch_deg x z y_start y_end"""
import sys
import numpy as np
import rclpy
sys.path.insert(0, "/workspace/tools")
from robot import Robot, mat_to_quat, LIMITS


def R_of(yaw, pitch):
    """hand z = approach a (yaw about world z from +y, pitch down), hand x ~ -world z."""
    cy, sy, cp, sp = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch)
    a = np.array([-sy * cp, cy * cp, -sp])
    xh = np.array([-sy * sp, cy * sp, cp]) * -1.0        # ~ -z, tilted with pitch
    yh = np.cross(a, xh)
    return np.stack([xh, yh, a], 1)


def main():
    yaw, pitch, x, z, y0, y1 = map(float, sys.argv[1:7])
    rclpy.init(); r = Robot()
    R = R_of(np.deg2rad(yaw), np.deg2rad(pitch)); q_ = mat_to_quat(R)
    seed = None
    last = None
    for y in np.arange(y0, y1 - 1e-9, -0.01 if y1 < y0 else 0.01):
        q = r.ik([x, y, z], q_, seed=seed, attempts=8)
        if q is None:
            print(f"yaw {yaw:+.0f} pitch {pitch:+.0f}  y={y:.3f}: FAIL", flush=True)
            break
        last = (y, q)
        seed = q
        marg = min(min(v - lo, hi - v) for v, (lo, hi) in zip(q, LIMITS))
        print(f"yaw {yaw:+.0f} pitch {pitch:+.0f}  y={y:.3f}: ok  q={np.round(q, 2).tolist()} limit-margin {marg:.2f}", flush=True)
    if last:
        print("LAST_OK", round(last[0], 3), np.round(last[1], 4).tolist())
    r.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
