#!/usr/bin/env python3
"""Door helpers: geometry of the push poses + the sweep."""
import sys
import numpy as np
from arm import Arm, R_from_axes
from scene import HINGE, birdview_cloud, door_state

Z_TIPS = 1.06        # push height: fingers overlap door top (1.107), hand body clears box top
Z_CLEAR = 1.20       # tips height when travelling above the box (top 1.107)


def d_n(alpha_deg):
    a = np.radians(alpha_deg)
    d = np.array([np.cos(a), -np.sin(a), 0.0])     # door direction from hinge
    n = np.array([np.sin(a), np.cos(a), 0.0])      # inner normal (+y when closed)
    return d, n


def push_pose(alpha_deg, r, side, off=0.02, z=Z_TIPS):
    """Tips point + hand rotation for pushing the door at angle alpha.
    side=+1: fingers on the inner face (opening), -1: on the outer face (closing)."""
    d, n = d_n(alpha_deg)
    tips = np.array([HINGE[0], HINGE[1], z]) + r * d + side * off * n
    R = R_from_axes(z=[0, 0, -1], y=d)
    return tips, R


def hand_from_tips(tips, R):
    return np.asarray(tips) - 0.1034 * R[:, 2]


def sweep(a, alphas, rs, side, off=0.02, secs_per=0.7):
    """IK every waypoint (seeded by the previous), check continuity, run."""
    q = a.q(); pts = []
    for al, r in zip(alphas, rs):
        tips, R = push_pose(al, r, side, off)
        sol = a.solve_ik(hand_from_tips(tips, R), R, seed=q)
        if sol is None:
            print(f"  no IK at alpha={al:.1f}"); return False
        jump = np.abs(sol - q).max()
        if jump > 0.6:
            print(f"  joint jump {jump:.2f} at alpha={al:.1f}; abort"); return False
        pts.append(sol); q = sol
    code, err = a.traj(pts, [secs_per * (i + 1) for i in range(len(pts))])
    return err < 0.05


if __name__ == "__main__":
    pass
