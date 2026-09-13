#!/usr/bin/env python3
"""Body-grasp the lying red mug near its bottom end, stand it up, place it on the plate.

Stages (run one or more, in order):  plan approach grasp rotate carry place
State between stages is kept in grasp.json.
"""
import json, sys, subprocess
import numpy as np, rclpy
from arm import Arm, qmul, quat_axis, down_closing_along, quat_to_R
from mugpose import cloud, measure, report

PLATE = np.array([0.121, -0.004])
GRASP_FROM_BOTTOM = 0.028      # pad centre this far from the mug bottom (along axis)
BODY_R = 0.035                 # body radius near the bottom end
TABLE = 0.425
PLATE_RIM_Z = 0.456
PALM_HALF = 0.030              # palm half-thickness (hand x)
STATE = "grasp.json"


def load():
    s = json.load(open(STATE)); return {k: (np.array(v) if isinstance(v, list) else v) for k, v in s.items()}


def save(s):
    json.dump({k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in s.items()}, open(STATE, "w"), indent=1)


def plan():
    subprocess.run([sys.executable, "/workspace/locate.py", "birdview"], check=True, stdout=subprocess.DEVNULL)
    P = cloud(); mp = measure(P); report(mp)
    c, u, v = mp["center"], mp["u"], mp["v"]
    along = -mp["length"] / 2 + GRASP_FROM_BOTTOM
    # lateral centre + axis height from body points in the grasp slice
    m = (P[0] > 0.12) & (P[0] < 0.40) & (P[1] > -0.25) & (P[1] < 0.25) & (P[2] > 0.48) & (P[2] < 0.6)
    q = np.c_[P[0][m], P[1][m]] - c; al = q @ u; ac = q @ v; z = P[2][m]
    s = (al > along - 0.012) & (al < along + 0.012)
    lat = 0.5 * (ac[s].min() + ac[s].max()); width = ac[s].max() - ac[s].min(); top = z[s].max()
    axis_z = top - BODY_R
    g = c + along * u + lat * v
    theta = mp["angle_deg"]
    st = dict(c=c, u=u, v=v, theta=theta, g=g, gz=axis_z + 0.003, width=width, top=top, along=along)
    print("grasp slice: along %.3f lateral centre %.4f width %.3f top %.3f -> axis z %.3f" % (along, lat, width, top, axis_z))
    print("grasp TCP (%.4f,%.4f,%.4f), closing along v (yaw %.1f)" % (*g, st["gz"], theta + 90))
    save(st)
    return st


def q_grasp(st):
    return down_closing_along(st["theta"] + 90)


def approach(a, st):
    Q = q_grasp(st); g = st["g"]
    a.gripper(0.04)
    print("== hover"); a.goto([*g, 0.60], Q, seconds=4.0, max_jump=2.5)
    print("== pre-grasp (fingertips 2 cm above axis)"); a.goto([*g, st["gz"] + 0.022], Q, seconds=3.0)
    subprocess.run([sys.executable, "/workspace/locate.py", "agentview"], check=True, stdout=subprocess.DEVNULL)
    print("agentview captured for check")


def grasp(a, st):
    Q = q_grasp(st); g = st["g"]
    print("== descend to grasp height"); a.goto([*g, st["gz"]], Q, seconds=3.0)
    f = a.gripper(0.0)
    gap = abs(f[0]) + abs(f[1])
    print("finger gap %.4f (expect ~%.3f)" % (gap, st["width"]))
    if gap < 0.055 or gap > 0.078:
        raise RuntimeError("grasp looks wrong (gap %.4f)" % gap)
    print("== lift 1.5 cm"); a.goto([*g, st["gz"] + 0.015], Q, seconds=2.0)
    f = a.fingers(); print("fingers after small lift", f)
    if abs(f[0]) + abs(f[1]) < 0.055:
        raise RuntimeError("lost the mug")


def rotate(a, st):
    """Rotate about the closing axis so the mug bottom points down; hand ends pointing +u."""
    Q0 = q_grasp(st); g = st["g"]; v = st["v"]; z = st["gz"] + 0.015
    for deg in (-30, -60, -90):
        Q = qmul(quat_axis([v[0], v[1], 0.0], deg), Q0)
        print(f"== rotate {deg} deg"); a.goto([*g, z], Q, seconds=3.0)
        print("fingers", a.fingers())
    st["q_up"] = qmul(quat_axis([v[0], v[1], 0.0], -90), Q0); save(st)
    print("== lift to 0.60"); a.goto([*g, 0.60], st["q_up"], seconds=3.0)
    subprocess.run([sys.executable, "/workspace/locate.py", "agentview"], check=True, stdout=subprocess.DEVNULL)


CARRY_YAW = 30.0   # extra yaw about world z; IK probe: feasible without branch change, wrist clears the white mug


def carry(a, st):
    """Yaw a little (+y) so the wrist stays clear of the white mug, then move over the plate."""
    Qx = qmul(quat_axis([0, 0, 1], CARRY_YAW), st["q_up"]); st["q_x"] = Qx; save(st)
    g = st["g"]
    print("== yaw"); a.goto([*g, 0.60], Qx, seconds=3.0, max_jump=1.2)
    print("== over plate"); a.goto([*PLATE, 0.60], Qx, seconds=3.5, max_jump=1.2)
    print("fingers", a.fingers())


def place(a, st):
    Qx = st["q_x"]
    hz = quat_to_R(*Qx)[:, 2]                         # hand z (horizontal, pointing from the palm to the mug)
    tcp_z = PLATE_RIM_Z + PALM_HALF + 0.004          # palm bottom 4 mm above the plate rim
    print("mug bottom will be %.3f above plate top" % (tcp_z - GRASP_FROM_BOTTOM - 0.444))
    print("== lower"); a.goto([*PLATE, tcp_z], Qx, seconds=3.5)
    print("== release"); a.gripper(0.04)
    back = np.array([PLATE[0], PLATE[1], tcp_z]) - 0.07 * np.array([hz[0], hz[1], 0.0])
    print("== retreat along -hand z"); a.goto(back, Qx, seconds=2.5)
    print("== up"); a.goto([back[0], back[1], 0.62], Qx, seconds=3.0)
    subprocess.run([sys.executable, "/workspace/locate.py", "agentview"], check=True, stdout=subprocess.DEVNULL)


if __name__ == "__main__":
    stages = sys.argv[1:]
    st = plan() if "plan" in stages else load()
    todo = [s for s in stages if s != "plan"]
    if todo:
        a = Arm()
        for s in todo:
            globals()[s](a, st)
        rclpy.shutdown()
