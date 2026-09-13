#!/usr/bin/env python3
"""Shift the upright pot B inward on the plate with a side grasp.
Usage: python3 moveB.py [--dry]  (phases: pick | place | all)"""
import math, sys
import numpy as np
from task import Task, HOME

C_B = np.array([0.128, 0.034])       # measured centre of upright pot B
S_B = np.array([0.16, 0.078])        # target centre
PHI = math.radians(-45)              # approach azimuth (hand z direction)
GZ = 0.998                           # grasp TCP z: ~6.8 cm above pot bottom (bottom on plate 0.930)
BACK = 0.01
DIR = np.array([math.cos(PHI), math.sin(PHI), 0.0])


class MoveB(Task):
    def __init__(self, dry):
        super().__init__(dry)
        self.obstacles = {"potA_lying": np.array([0.03, -0.06, 1.01]), "knob": np.array([0.036, 0.03, 0.959]),
                          "potB": np.array([*C_B, 1.09])}

    def open_full(self):
        for k in range(6):
            f = self.gripper(0.04)
            if self.dry or (f[0] > 0.039 and -f[1] > 0.039):
                break
            print("   fingers not fully open yet, re-issuing", flush=True)

    def pick(self):
        tcp = np.array([*(C_B - BACK * DIR[:2]), GZ])
        pre = tcp - 0.07 * DIR
        self.open_full()
        if np.abs(self.cur() - HOME).max() > 0.05:
            self.home()
        self.go([[pre[0], pre[1], 1.15]], [PHI], 4.0, "pickB: high pre-grasp")
        self.go([pre], [PHI], 3.0, "pickB: descend")
        self.go([tcp], [PHI], 2.0, "pickB: advance")
        f = self.gripper(0.0)
        self.obstacles.pop("potB")
        self.go([[tcp[0], tcp[1], 1.10]], [PHI], 3.0, "pickB: lift")
        return f

    def place(self):
        start = np.array([*(C_B - BACK * DIR[:2]), 1.10])
        end = np.array([*(S_B - BACK * DIR[:2]), 1.10])
        self.go([start * (1 - t) + end * t for t in (0.5, 1.0)], [PHI, PHI], 4.0, "placeB: transport")
        low = np.array([end[0], end[1], GZ + 0.005])
        self.go([low], [PHI], 3.0, "placeB: lower")
        self.open_full()
        back = low - 0.06 * DIR
        self.go([back], [PHI], 2.0, "placeB: retreat")
        self.go([[back[0], back[1], 1.15]], [PHI], 2.0, "placeB: rise")
        self.home()


if __name__ == "__main__":
    dry = "--dry" in sys.argv
    a = [x for x in sys.argv[1:] if not x.startswith("--")] or ["all"]
    t = MoveB(dry)
    if a[0] in ("pick", "all"):
        t.pick()
    if a[0] in ("place", "all"):
        t.place()
    if not dry:
        print("fingers:", t.r.fingers())
