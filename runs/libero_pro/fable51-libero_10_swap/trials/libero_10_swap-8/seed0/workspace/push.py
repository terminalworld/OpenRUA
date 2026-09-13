#!/usr/bin/env python3
"""Push a lying pot sideways with the closed fingertip: approach its flank horizontally at height z,
detect contact (TCP shortfall), then keep pushing to target x while reading the TCP each step.
Usage: python3 push.py  px py pz  x_target [tilt_deg=30] [--dry]
  (px,py,pz): point ~3 cm outside the pot flank on the +x side, at pushing height; pushes toward -x.
"""
import math, sys
import numpy as np
from recover import Recover
from task import TCP_OFF


class Push(Recover):
    def push(self, start, x_target, tilt=math.radians(30), step=0.01):
        # hand z leans toward +x by `tilt` from vertical (wrist stays on the robot side), fingers along y
        z = np.array([math.sin(tilt), 0.0, -math.cos(tilt)]); y = np.array([0.0, 1.0, 0.0]); x = np.cross(y, z)
        R = np.column_stack([x, y, z])
        self.gripper(0.0)
        tcp = np.asarray(start, float)
        self.go_R([tcp + [0, 0, 0.10]], [R], 4.0, "push: above")
        tip = self.go_R([tcp], [R], 3.0, "push: descend", retry=False)
        if not self.dry:
            print(f"   descend: TCP z {tip[2]:.4f} vs cmd {tcp[2]:.4f} (shortfall {(tip[2]-tcp[2])*1000:.1f} mm)", flush=True)
            if tip[2] - tcp[2] > 0.003:
                print("   !! blocked while descending", flush=True)
                self.go_R([tip + [0, 0, 0.10]], [R], 3.0, "push: rise"); self.open_full(); return False
        contact = False
        while tcp[0] - x_target > 1e-6:
            tcp = tcp - [step, 0, 0]
            tip = self.go_R([tcp], [R], 1.0, f"push: x={tcp[0]:.3f}", retry=False)
            if self.dry:
                tip = tcp
            lag = tip[0] - tcp[0]
            print(f"   TCP {np.round(tip, 4)} lag {lag*1000:.1f} mm", flush=True)
            if not contact and lag > 0.003:
                contact = True
                print("   contact", flush=True)
        if not contact and not self.dry:
            print("   !! no contact detected over the whole push", flush=True)
        self.go_R([tip + [0.02, 0, 0]], [R], 1.5, "push: back off")
        self.go_R([tip + [0.02, 0, 0.10]], [R], 3.0, "push: rise")
        self.open_full()
        return contact


if __name__ == "__main__":
    dry = "--dry" in sys.argv
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    t = Push(dry)
    tilt = math.radians(float(a[4])) if len(a) > 4 else math.radians(30)
    ok = t.push([float(a[0]), float(a[1]), float(a[2])], float(a[3]), tilt)
    print("PUSH", "contact" if ok else "no-contact")
    if not dry:
        print("fingers:", t.r.fingers())
