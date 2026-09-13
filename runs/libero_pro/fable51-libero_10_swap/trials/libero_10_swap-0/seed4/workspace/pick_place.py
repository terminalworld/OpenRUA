#!/usr/bin/env python3
"""Pick one object top-down and place it in the basket, verifying each step.

Usage: python3 -u pick_place.py <name> <x> <y> <z_top> <grasp_tcp_z> [--stop-after-pre]
"""
import subprocess
import sys

import numpy as np

from arm import Arm, Q_DOWN

BASKET = (0.005, 0.25)
CARRY_Z = 0.76           # TCP height while carrying (basket rim is 0.63)
BASKET_DROP_Z = 0.72     # TCP height when releasing over basket


def snap(cam, out):
    subprocess.run([sys.executable, "/workspace/tools/perception/cam_snap.py", cam, out],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=120)


def main():
    name, x, y, z_top, gz = sys.argv[1], *map(float, sys.argv[2:6])
    stop_pre = "--stop-after-pre" in sys.argv
    a = Arm()
    pre_z = z_top + 0.13
    print(f"[{name}] target ({x:.3f},{y:.3f}) top={z_top:.3f} grasp_z={gz:.3f}")

    print("1. open gripper")
    gap = a.gripper(0.04)
    assert gap > 0.07, "gripper did not open"

    print("2. move to pre-grasp")
    tcp = a.move_tcp((x, y, pre_z), Q_DOWN, seconds=4.0)
    assert tcp is not None and np.linalg.norm(tcp - (x, y, pre_z)) < 0.01, "pre-grasp not reached"
    snap("robot0_eye_in_hand", f"{name}_pre_eih.png")
    print("   saved", f"{name}_pre_eih.png")
    if stop_pre:
        return

    print("3. descend")
    steps = [(x, y, z) for z in np.linspace(pre_z, gz, 4)[1:-1]]
    tcp = a.move_tcp((x, y, gz), Q_DOWN, seconds=3.0, via=steps)
    assert tcp is not None and np.linalg.norm(tcp - (x, y, gz)) < 0.01, "grasp pose not reached"
    print("   wrench", np.round(a.wrench(), 2))

    print("4. close gripper")
    gap = a.gripper(0.0)
    print(f"   gap={gap:.4f} (>0.02 means something is held)")
    assert gap > 0.02, "closed on air"

    print("5. lift")
    tcp = a.move_tcp((x, y, CARRY_Z), Q_DOWN, seconds=3.0, via=[(x, y, gz + 0.08)])
    gap = a.finger_gap()
    print(f"   gap after lift={gap:.4f}")
    assert gap > 0.02, "object dropped during lift"
    snap("agentview", f"{name}_lifted.png")

    print("6. move over basket")
    tcp = a.move_tcp((BASKET[0], BASKET[1], CARRY_Z), Q_DOWN, seconds=4.0)
    assert tcp is not None and np.linalg.norm(tcp - (*BASKET, CARRY_Z)) < 0.01, "basket pose not reached"
    gap = a.finger_gap()
    print(f"   gap over basket={gap:.4f}")
    assert gap > 0.02, "object dropped in transit"
    snap("robot0_eye_in_hand", f"{name}_over_basket_eih.png")

    print("7. lower a bit and release")
    a.move_tcp((BASKET[0], BASKET[1], BASKET_DROP_Z), Q_DOWN, seconds=2.0)
    gap = a.gripper(0.04)
    print(f"   gap after open={gap:.4f}")

    print("8. retreat up")
    a.move_tcp((BASKET[0], BASKET[1], 0.85), Q_DOWN, seconds=2.0)
    snap("agentview", f"{name}_placed.png")
    snap("robot0_eye_in_hand", f"{name}_placed_eih.png")
    print(f"[{name}] DONE")


if __name__ == "__main__":
    main()
