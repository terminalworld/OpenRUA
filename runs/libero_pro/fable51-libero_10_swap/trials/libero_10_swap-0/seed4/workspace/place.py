#!/usr/bin/env python3
"""Finish a place: carry the held object over the basket, release, retreat.
Usage: python3 -u place.py <name>"""
import subprocess
import sys

import numpy as np

from arm import Arm, Q_DOWN

BASKET = (0.005, 0.25)
CARRY_Z, DROP_Z = 0.76, 0.72


def snap(cam, out):
    subprocess.run([sys.executable, "/workspace/tools/perception/cam_snap.py", cam, out],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=120)


def goto(a, xyz, seconds, tries=3):
    for i in range(tries):
        tcp = a.move_tcp(xyz, Q_DOWN, seconds=seconds)
        if tcp is not None and np.linalg.norm(tcp - xyz) < 0.01:
            return tcp
        print(f"   retry {i + 1}: off by {np.linalg.norm(tcp - xyz):.3f}")
    raise SystemExit("could not reach " + str(xyz))


def main():
    name = sys.argv[1]
    a = Arm()
    assert a.finger_gap() > 0.02, "nothing held"
    print("6. move over basket")
    goto(a, (*BASKET, CARRY_Z), 4.0)
    gap = a.finger_gap()
    print(f"   gap over basket={gap:.4f}")
    assert gap > 0.02, "object dropped in transit"
    snap("robot0_eye_in_hand", f"{name}_over_basket_eih.png")
    print("7. lower and release")
    goto(a, (*BASKET, DROP_Z), 2.0)
    gap = a.gripper(0.04)
    print(f"   gap after open={gap:.4f}")
    print("8. retreat up")
    goto(a, (*BASKET, 0.86), 2.5)
    snap("agentview", f"{name}_placed.png")
    snap("robot0_eye_in_hand", f"{name}_placed_eih.png")
    print(f"[{name}] PLACED")


if __name__ == "__main__":
    main()
