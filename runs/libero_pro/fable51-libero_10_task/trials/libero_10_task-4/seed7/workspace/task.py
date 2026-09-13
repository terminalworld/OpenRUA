#!/usr/bin/env python3
"""Phase-driven pick & place. Usage: python3 task.py <phase> [args]
Phases: pose | pregrasp X Y ZTOP YAW | descend Z | grasp | lift Z | carry X Y Z | place Z | release | retreat Z
World frame; X,Y = rim grasp point (TCP), YAW = hand x-axis yaw (fingers open along yaw+90deg)."""
import math, sys, json
import numpy as np
from arm import Arm, fmt, TCP, LIM

def ik_tcp(a, tcp, yaw, seed=None):
    hand = np.array(tcp, float) + np.array([0, 0, TCP])
    return a.ik_topdown(hand, yaw, seed=seed)

def cart_move(a, tcp_goal, yaw, seconds=None, steps=None):
    """Move TCP straight to tcp_goal keeping top-down orientation; IK per waypoint, one trajectory."""
    p = a.hand_pose(); start = p["tcp"]
    goal = np.array(tcp_goal, float)
    dist = np.linalg.norm(goal - start)
    n = steps or max(2, int(dist / 0.05) + 1)
    seconds = seconds or max(2.5, dist * 10)
    seed = p["q"]; via = []
    for i in range(1, n + 1):
        wp = start + (goal - start) * i / n
        q = ik_tcp(a, wp, yaw, seed=seed)
        if q is None:
            print(f"IK failed at waypoint {wp}"); return False
        if max(abs(x - y) for x, y in zip(q, seed)) > 1.0:
            print(f"WARNING big joint jump at waypoint {i}: {np.round(np.array(q)-np.array(seed),2)}")
        via.append(q); seed = q
    ok = a.move_to(via[-1], seconds) if n == 1 else _move_via(a, via, seconds)
    for _ in range(3):
        p = a.hand_pose(); err = np.linalg.norm(p["tcp"] - goal)
        dyaw = abs((yaw - p["yaw"] + math.pi) % (2 * math.pi) - math.pi)
        print(f"  arrived: {fmt(p)}  tcp_err={err*1000:.1f}mm yaw_err={math.degrees(dyaw):.1f}deg")
        if err < 0.008 and dyaw < 0.05:
            return True
        print("  re-sending final point")
        a.move_to(via[-1], 2.5)
    return False

def _move_via(a, via, seconds):
    code, err = a.move(via[-1], seconds, via=via[:-1])
    print(f"  move(via {len(via)}): code={code} max_joint_err={err:.4f}")
    if err > 0.02:
        code, err = a.move(via[-1], 2.0)
        print(f"  re-send final: code={code} max_joint_err={err:.4f}")
    return err < 0.02

def main():
    a = Arm()
    ph = sys.argv[1]; args = [float(x) for x in sys.argv[2:]]
    p = a.hand_pose(); print("start:", fmt(p))
    if ph == "pose":
        print("wrench:", a.wrench().round(2))
    elif ph == "goto":  # goto X Y Z YAW(deg): TCP target, straight line
        x, y, z, yaw = args; cart_move(a, [x, y, z], math.radians(yaw))
    elif ph == "grasp":
        a.gripper(0.0)
        print("wrench:", a.wrench().round(2))
    elif ph == "release":
        a.gripper(0.04)
    elif ph == "servo":  # servo VX VY VZ TICKS (m/s, base frame)
        vx, vy, vz, t = args; print(fmt(a.servo(vx, vy, vz, int(t))))
    print("end:", fmt(a.hand_pose()))

if __name__ == "__main__":
    main()
