#!/usr/bin/env python3
"""Run one manipulation stage. Usage:
  stage.py pregrasp X Y Z_TCP        open gripper, move TCP (world) above target, top-down
  stage.py goto X Y Z_TCP [secs]     move TCP (world) top-down, no gripper change
  stage.py grasp Z_TCP               descend to Z at current XY, close gripper, report gap
  stage.py lift Z_TCP                rise to Z at current XY, report gap
  stage.py release                   open gripper
  stage.py pose                      print current TCP world pose and finger gap
Writes progress to stdout (run with python3 -u > file).
"""
import sys
import numpy as np
from robot import Robot, log

Q_DOWN = np.array([1.0, 0.0, 0.0, 0.0])  # hand z -> world -z, fingers open along world y


def main():
    cmd = sys.argv[1]
    r = Robot()
    tcp, q = r.tcp_world()
    log(f"start tcp={tcp.round(4)} q={q.round(4)} gap={r.finger_gap():.4f}")
    if cmd == "pose":
        return
    if cmd == "release":
        r.gripper(0.04)
        return
    if cmd == "pregrasp":
        x, y, z = map(float, sys.argv[2:5])
        r.gripper(0.04)
        res = r.move_tcp_world([x, y, z], Q_DOWN, 4.0)
        if res is None:
            raise SystemExit("IK failed")
    elif cmd == "goto":
        x, y, z = map(float, sys.argv[2:5])
        secs = float(sys.argv[5]) if len(sys.argv) > 5 else 4.0
        res = r.move_tcp_world([x, y, z], Q_DOWN, secs)
        if res is None:
            raise SystemExit("IK failed")
    elif cmd == "grasp":
        z = float(sys.argv[2])
        res = r.move_tcp_world([tcp[0], tcp[1], z], Q_DOWN, 3.0)
        if res is None:
            raise SystemExit("IK failed")
        gap = r.gripper(0.0)
        log(f"GRASP gap={gap:.4f} ({'holding' if gap > 0.004 else 'EMPTY'})")
    elif cmd == "lift":
        z = float(sys.argv[2])
        res = r.move_tcp_world([tcp[0], tcp[1], z], Q_DOWN, 3.0)
        if res is None:
            raise SystemExit("IK failed")
        log(f"after lift gap={r.finger_gap():.4f}")
    tcp, q = r.tcp_world()
    log(f"end tcp={tcp.round(4)} q={q.round(4)} gap={r.finger_gap():.4f}")


if __name__ == "__main__":
    main()
