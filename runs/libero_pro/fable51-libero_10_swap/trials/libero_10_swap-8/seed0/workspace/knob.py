#!/usr/bin/env python3
"""Recover a lying moka pot by its lid knob: pinch the knob, lift (the pot swings upright and hangs
from the knob), carry over the spot, lower until the bottom touches, open, rise.
Usage:
  python3 knob.py pick  kx ky kz  ax ay  gamma_deg sgn pitch_deg [--dry]   # knob centre, pot axis (bottom->lid)
  python3 knob.py place sx sy drop_m [--dry]      # drop_m: TCP z - pot bottom z while hanging
  python3 knob.py rise | home
"""
import math, sys
import numpy as np
from recover import Recover, opt, PLATE_TOP
from task import TCP_OFF
from kin import fk_hand


class Knob(Recover):
    def pick(self, knob, a, gam, sgn, pitch, lift=0.20):
        a = np.array([a[0], a[1], 0.0]); a /= np.linalg.norm(a)
        Z = np.array([0, 0, 1.0]); h = np.cross(a, Z)
        y = sgn * (math.cos(gam) * h + math.sin(gam) * Z)
        zd = -Z - (-Z @ y) * y; zd /= np.linalg.norm(zd)
        z = math.cos(pitch) * zd + math.sin(pitch) * a
        x = np.cross(y, z)
        R = np.column_stack([x, y, z])
        tcp = np.asarray(knob, float)
        pts = [tcp + s * y + t * z + u * x for s in (-0.051, 0.051) for t in (0.0, 0.01) for u in (-0.009, 0.009)]
        print("   hand z:", np.round(z, 3), "fingers:", np.round(y, 3), "lowest finger corner:", np.round(min(pts, key=lambda v: v[2]), 3), flush=True)
        self.open_full()
        self.go_R([tcp - 0.08 * z], [R], 4.0, "knob: above")
        self.go_R([tcp], [R], 3.0, "knob: descend", retry=False)
        f = self.gripper(0.0)
        if not self.dry:
            w = f[0] - f[1]
            print(f"   pinch width {w*100:.2f} cm", flush=True)
            if w < 0.004:
                print("   !! nothing in the fingers", flush=True)
                return False
        self.go_R([tcp + [0, 0, 0.5 * lift], tcp + [0, 0, lift]], [R, R], 6.0, "knob: lift")
        return True

    def drag(self, target_xy, seconds=8.0):
        """Slide the lying pot along the plate by pulling the pinched knob horizontally."""
        R = self.hand_R()
        pos = fk_hand(self.q)[:3, 3] if self.dry else self.r.fk()[0]
        tip0 = pos + R[:, 2] * TCP_OFF
        end = np.array([target_xy[0], target_xy[1], tip0[2]])
        wps = [tip0 * (1 - s) + end * s for s in (0.25, 0.5, 0.75, 1.0)]
        self.go_R(wps, [R] * 4, seconds, "knob: drag")
        if not self.dry:
            f = self.r.fingers(); print(f"   pinch width after drag {(f[0]-f[1])*100:.2f} cm", flush=True)
        self.open_full()
        self.go_R([end + [0, 0, 0.12]], [R], 3.0, "knob: rise")

    def roll(self, crest, dx, press=0.03, step=0.01):
        """Roll a lying pot (axis along y) toward -x: closed fingertip edge lowered onto the crest until
        contact (TCP shortfall), pushed `press` further, then dragged along x in `step` increments.
        Hand z = (0.707,0,-0.707), fingers along y; the block's lowest edge is ~1.27 cm below the TCP."""
        z = np.array([1.0, 0.0, -1.0]) / math.sqrt(2); y = np.array([0.0, 1.0, 0.0]); x = np.cross(y, z)
        R = np.column_stack([x, y, z])
        self.gripper(0.0)
        tcp = np.array([crest[0], crest[1], crest[2] + 0.0127 + 0.012])   # nominal edge 1.2 cm above crest
        self.go_R([tcp - 0.08 * z], [R], 4.0, "roll: above")
        tip = self.go_R([tcp], [R], 3.0, "roll: start", retry=False)
        touched = None
        for k in range(10):                                              # creep down 3 mm at a time
            tcp = tcp - [0, 0, 0.003]
            tip = self.go_R([tcp], [R], 1.0, f"roll: creep {k}", retry=False)
            if self.dry:
                tip = tcp
            short = tip[2] - tcp[2]
            print(f"   creep: TCP z {tip[2]:.4f} vs cmd {tcp[2]:.4f} (shortfall {short*1000:.1f} mm)", flush=True)
            if short > 0.0015:
                touched = tip[2]; break
        if touched is None and not self.dry:
            print("   !! never touched the pot", flush=True)
            self.go_R([tip + [0, 0, 0.10]], [R], 3.0, "roll: rise"); self.open_full(); return False
        z_touch = touched if touched else tcp[2]
        while True:                                              # deepest reachable press command
            tcp = np.array([tcp[0], tcp[1], z_touch - press])    # soft arm: deep command = real normal force
            try:
                tip = self.go_R([tcp], [R], 2.0, f"roll: press {press*1000:.0f} mm", retry=False); break
            except RuntimeError:
                press -= 0.005
                if press < 0.005:
                    print("   !! press point unreachable", flush=True); self.open_full(); return False
        if not self.dry:
            print(f"   press: TCP z {tip[2]:.4f} (touch {z_touch:.4f}, cmd {tcp[2]:.4f})", flush=True)
        n = int(round(abs(dx) / step)); sgn = -1 if dx < 0 else 1
        for k in range(1, n + 1):
            tgt = tcp + [sgn * k * step, 0, 0]
            tip = self.go_R([tgt], [R], 1.0, f"roll: drag {k}/{n}", retry=False)
            if self.dry:
                tip = tgt
            else:
                print(f"   TCP {np.round(tip, 4)} (cmd {np.round(tgt, 4)}) lag x {(tip[0]-tgt[0])*1000:+.1f} mm, z above touch {(tip[2]-z_touch)*1000:+.1f} mm", flush=True)
                if tip[2] < z_touch - 0.012:
                    print("   !! finger dropped past the crest, aborting drag", flush=True)
                    break
        self.go_R([tip + [0, 0, 0.10]], [R], 3.0, "roll: rise")
        self.open_full()
        return True

    def place(self, spot, drop):
        R = self.hand_R()
        pos = fk_hand(self.q)[:3, 3] if self.dry else self.r.fk()[0]
        tip0 = pos + R[:, 2] * TCP_OFF
        z_touch = PLATE_TOP + drop
        high = np.array([spot[0], spot[1], tip0[2]])
        self.go_R([tip0 * 0.5 + high * 0.5, high], [R, R], 6.0, "knob place: transport")
        low = np.array([spot[0], spot[1], z_touch + 0.015])
        self.go_R([low], [R], 4.0, "knob place: lower")
        tip = low.copy()
        while low[2] > z_touch - 0.012 + 1e-6:
            low = low - [0, 0, 0.004]
            tip = self.go_R([low], [R], 1.0, "knob place: creep", retry=False)
            if not self.dry and tip[2] > low[2] + 0.0025:
                print(f"   touchdown: TCP z {tip[2]:.4f} vs cmd {low[2]:.4f}", flush=True)
                break
        self.open_full()
        if self.dry:
            tip = low
        self.go_R([[tip[0], tip[1], tip[2] + 0.15]], [R], 3.0, "knob place: rise")


if __name__ == "__main__":
    dry = "--dry" in sys.argv
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    t = Knob(dry)
    if a[0] == "pick":
        ok = t.pick([float(a[1]), float(a[2]), float(a[3])], [float(a[4]), float(a[5])],
                    math.radians(float(a[6])), int(a[7]), math.radians(float(a[8])))
        print("PICK", "OK" if ok else "FAILED")
    elif a[0] == "drag":   # pick (no lift) then pull the knob horizontally to (tx, ty), release, rise
        ok = t.pick([float(a[1]), float(a[2]), float(a[3])], [float(a[4]), float(a[5])],
                    math.radians(float(a[6])), int(a[7]), math.radians(float(a[8])), lift=0.0)
        if ok:
            t.drag((float(a[9]), float(a[10])))
        else:
            t.open_full()
    elif a[0] == "roll":   # roll cx cy cz dx [--press=m]
        press = float(next((x[8:] for x in sys.argv if x.startswith("--press=")), 0.03))
        t.roll([float(a[1]), float(a[2]), float(a[3])], float(a[4]), press=press)
    elif a[0] == "place":
        t.place((float(a[1]), float(a[2])), float(a[3]))
    elif a[0] == "rise":
        R = t.hand_R(); pos = t.r.fk()[0]; tip = pos + R[:, 2] * TCP_OFF
        t.go_R([[tip[0], tip[1], tip[2] + 0.15]], [R], 3.0, "rise")
    elif a[0] == "home":
        t.home()
    if not dry:
        print("fingers:", t.r.fingers())
