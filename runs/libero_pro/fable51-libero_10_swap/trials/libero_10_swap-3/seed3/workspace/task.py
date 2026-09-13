"""Stages of: put the black bowl in the bottom drawer, close it.
Usage: python3 -u task.py <stage> [stage...]   (stages run in order)"""
import sys
import numpy as np
from robot import *

# ---- scene measurements (world frame) ----
BOWL = np.array([-0.158, 0.056])       # bowl center xy
BOWL_RIM_Z = 0.951
BOWL_R = 0.055
GRASP_DY = 0.02                        # grasp the rim 2 cm toward +y from the -x point
GRASP_DX = -np.sqrt(BOWL_R**2 - GRASP_DY**2)   # ~ -0.0512
GRASP_Z = BOWL_RIM_Z - 0.018           # fingertips 1.8 cm below the rim
PLACE = np.array([-0.095, -0.140])     # bowl center xy inside the drawer
DRAWER_FLOOR_Z = 0.924
BOWL_BOTTOM_BELOW_TCP = 0.037   # measured after lift
RELEASE_Z = DRAWER_FLOOR_Z + BOWL_BOTTOM_BELOW_TCP + 0.008
SAFE_Z = 1.12
Q = Q_DOWN_FX  # pointing down, fingers along world X

pick_xy = BOWL + [GRASP_DX, GRASP_DY]
place_xy = PLACE - [0.051, 0.002]   # measured bowl-center offset from TCP after lift


def report(r, label):
    p, q = r.tcp()
    print(f"[{label}] TCP world = {p.round(4)}  fingers = {np.round(r.fingers(), 4)}")
    return p


def approach(r):
    print("approach above pick point", pick_xy)
    r.move_tcp([*pick_xy, 1.03], Q, secs=4.0)
    report(r, "above bowl")


def descend(r):
    p = report(r, "before descend")
    r.move_tcp_line(p, [*pick_xy, GRASP_Z], Q, n=3, secs=2.5)
    report(r, "at grasp depth")


def grasp(r):
    r.gripper(0.0)
    report(r, "closed")


def lift(r):
    p = report(r, "before lift")
    r.move_tcp_line(p, [p[0], p[1], SAFE_Z], Q, n=3, secs=2.5)
    report(r, "lifted")


def transfer(r):
    print("transfer above place point", place_xy)
    r.move_tcp([*place_xy, SAFE_Z], Q, secs=4.0)
    report(r, "above drawer")


def lower(r):
    p = report(r, "before lower")
    r.move_tcp_line(p, [*place_xy, RELEASE_Z], Q, n=4, secs=3.0)
    report(r, "at release height")


def release(r):
    r.gripper(0.04)
    report(r, "opened")


def retreat(r):
    p = report(r, "before retreat")
    r.move_tcp_line(p, [p[0], p[1], SAFE_Z], Q, n=3, secs=2.5)
    report(r, "retreated")


STAGES = dict(approach=approach, descend=descend, grasp=grasp, lift=lift,
              transfer=transfer, lower=lower, release=release, retreat=retreat)

if __name__ == "__main__":
    r = Robot()
    print("pick_xy", pick_xy.round(4), "place_xy", place_xy.round(4),
          "GRASP_Z", round(GRASP_Z, 4), "RELEASE_Z", round(RELEASE_Z, 4))
    for s in sys.argv[1:]:
        STAGES[s](r)
