#!/usr/bin/env python3
"""CLI over ctl.Ctl: a sequence of commands separated by '--'.
  pose                     print tcp pose + finger gap
  open | close             gripper fully open / closed
  tcp X Y Z [SEC]          IK move so the TCP is at world XYZ, top-down grip
  servo DX DY DZ TICKS     cartesian servo burst (m/s in panda_link0)
"""
import sys
import numpy as np
from ctl import Ctl, log

TOPDOWN = np.array([1.0, 0.0, 0.0, 0.0])  # hand x->+x, fingers along world y


def main():
    argv = sys.argv[1:]
    cmds, cur = [], []
    for a in argv:
        if a == "--":
            cmds.append(cur); cur = []
        else:
            cur.append(a)
    if cur:
        cmds.append(cur)
    c = Ctl()
    ok_all = True
    for cmd in cmds:
        name, args = cmd[0], cmd[1:]
        log(">>", " ".join(cmd))
        if name == "pose":
            p, q = c.tcp_pose()
            log(f"tcp {np.round(p, 4)} quat {np.round(q, 4)} gap {c.finger_gap():.4f}")
        elif name == "open":
            c.gripper(0.04)
        elif name == "close":
            c.gripper(0.0)
        elif name == "tcp":
            xyz = [float(x) for x in args[:3]]
            sec = float(args[3]) if len(args) > 3 else 3.0
            ok = c.move_tcp(xyz, TOPDOWN, sec)
            ok_all &= ok
            if not ok:
                log("!! move failed, stopping sequence"); break
        elif name == "servo":
            dx, dy, dz, ticks = float(args[0]), float(args[1]), float(args[2]), int(args[3])
            c.servo(dx, dy, dz, ticks)
        else:
            log("unknown", name)
    c.close()
    sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
