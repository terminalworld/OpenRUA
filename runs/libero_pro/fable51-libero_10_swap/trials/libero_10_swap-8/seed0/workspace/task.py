#!/usr/bin/env python3
"""Moka pot pick & place phases. Usage: python3 task.py <phase> [--dry]
Phases: approachA graspA liftA placeA  approachB graspB liftB placeB  home
All positions are TCP (fingertip pad centre) in world; hand origin = TCP - 0.1034*zhat.
"""
import math, sys, time
import numpy as np
import kin
from kin import fk_all, fk_hand, ik
from rob import Robot, R_to_quat, quat_to_R

kin.LIMITS = np.array([[-2.85, 2.85], [-1.72, 1.72], [-2.85, 2.85], [-3.0, -0.1], [-2.85, 2.85], [0.0, 3.7], [-2.85, 2.85]])
TCP_OFF = 0.1034
HOME = np.array([0, -0.161037389, 0, -2.44459747, 0, 2.2267522, 0.7853981633974483])

# scene (measured)
POTS = {"A": np.array([-0.196, -0.195]), "B": np.array([-0.057, 0.257])}
WAIST_Z = 0.965
SPOTS = {"A": (np.array([0.17, -0.008]), math.radians(45)),   # -y spot, yaw +45
         "B": (np.array([0.17, 0.072]), math.radians(-45))}   # +y spot, yaw -45
Z_CARRY = 1.10
Z_RELEASE = 1.01
GRASP_BACK = 0.01   # TCP sits 1 cm behind pot axis (palm clearance)
PRE = 0.07          # pre-grasp standoff along approach


def R_yaw(phi):
    c, s = math.cos(phi), math.sin(phi)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


R_POS = np.column_stack([[0, 0, -1], [0, 1, 0], [1, 0, 0]])  # hand x=-Z, y=+Y, z=+X


def hand_pose(tcp, phi):
    R = R_yaw(phi) @ R_POS
    zhat = R[:, 2]
    return np.asarray(tcp) - TCP_OFF * zhat, R_to_quat(R)


class Task:
    def __init__(self, dry):
        self.dry = dry
        self.r = None if dry else Robot()
        self.q = HOME.copy() if dry else self.r.arm()
        # xy + top z of things the hand must not sweep through (pots on table, knob, pots on stove)
        self.obstacles = {"potA": np.array([*POTS["A"], 1.053]), "potB": np.array([*POTS["B"], 1.053]),
                          "knob": np.array([0.036, 0.03, 0.959])}

    def cur(self):
        return self.q if self.dry else self.r.arm()

    def solve(self, tcp, phi, seed):
        pos, quat = hand_pose(tcp, phi)
        q = ik(pos, quat, seed, w_post=0.03, tries=12)
        if q is None:
            raise RuntimeError(f"IK failed for tcp={tcp} phi={math.degrees(phi):.0f}")
        return q

    def go(self, tcps, phis, seconds, label=""):
        """Cartesian waypoint list -> sequential IK -> one trajectory."""
        seed = self.cur()
        qs = []
        for tcp, phi in zip(tcps, phis):
            q = self.solve(tcp, phi, seed)
            qs.append(q); seed = q
        n = len(qs)
        times = [seconds * (i + 1) / n for i in range(n)]
        print(f"[{label}] {n} waypoints, {seconds}s", flush=True)
        for q, tcp in zip(qs, tcps):
            Ts = fk_all(q)
            tip = Ts[-1][:3, 3] + Ts[-1][:3, 2] * TCP_OFF
            print(f"   q={np.round(q, 3)} tcp={np.round(tip, 3)} minlinkz={min(T[2, 3] for T in Ts[2:]):.3f}", flush=True)
        # sweep check between consecutive configs
        prev = self.cur()
        for q in qs:
            for s in np.linspace(0, 1, 8)[1:]:
                qq = prev * (1 - s) + q * s
                Ts = fk_all(qq); tip = Ts[-1][:3, 3] + Ts[-1][:3, 2] * TCP_OFF
                if tip[2] < 0.93 or Ts[-1][2, 3] < 0.95:
                    print(f"   !! sweep low: tip z={tip[2]:.3f} hand z={Ts[-1][2,3]:.3f} at s={s:.2f}", flush=True)
                for nm, pt in self.obstacles.items():
                    for pname, p in (("tip", tip), ("hand", Ts[-1][:3, 3])):
                        if np.linalg.norm(p[:2] - pt[:2]) < 0.07 and p[2] < pt[2] + 0.01:
                            print(f"   !! sweep near {nm} ({pname} at {np.round(p,3)}) s={s:.2f}", flush=True)
            prev = q
        if self.dry:
            self.q = qs[-1]
            return
        # pace by the largest joint excursion (controller lags fast goals)
        dq = max(np.abs(np.diff(np.vstack([self.cur()] + qs), axis=0)).max(axis=1).sum(), 1e-3)
        scale = max(1.0, 2.0 * dq / seconds)
        times = [t * scale for t in times]
        code, err = self.r.move_joints(qs, times)
        for k in range(5):
            if code == 0 and err < 0.01:
                break
            print(f"   retrying final waypoint (code={code}, err={err:.4f})", flush=True)
            code, err = self.r.move_joints([qs[-1]], max(2.0, 2.0 * err))
        pos, quat = self.r.fk()
        R = quat_to_R(quat); tip = pos + R[:, 2] * TCP_OFF
        print(f"   TCP now {np.round(tip, 4)} (target {np.round(tcps[-1], 4)}) err={np.linalg.norm(tip - tcps[-1]):.4f}", flush=True)
        return tip

    def gripper(self, w):
        if self.dry:
            print(f"[gripper {w}]"); return
        return self.r.gripper(w)

    # ---- phases
    def home(self):
        if self.dry:
            self.q = HOME.copy(); return
        dq = np.abs(self.cur() - HOME).sum()
        code, err = self.r.move_joints([HOME], max(4.0, 2.0 * dq))
        for k in range(5):
            if code == 0 and err < 0.01:
                break
            print(f"   retrying HOME (code={code}, err={err:.4f})", flush=True)
            code, err = self.r.move_joints([HOME], max(2.0, 2.0 * err))

    def approach(self, P):
        p = POTS[P]
        far = np.array([p[0] - GRASP_BACK - PRE, p[1]])
        self.gripper(0.04)
        if np.abs(self.cur() - HOME).max() > 0.05:
            print(f"[approach{P}: via HOME]", flush=True)
            self.home()
        self.go([[far[0], far[1], 1.15]], [0.0], 4.0, f"approach{P}: high pre-grasp")
        self.go([[far[0], far[1], WAIST_Z]], [0.0], 3.0, f"approach{P}: descend")

    def grasp(self, P):
        p = POTS[P]
        self.go([[p[0] - GRASP_BACK, p[1], WAIST_Z]], [0.0], 2.0, f"grasp{P}: advance")
        f = self.gripper(0.0)
        return f

    def lift(self, P):
        p = POTS[P]
        self.obstacles.pop("pot" + P, None)   # it is in the hand now
        self.go([[p[0] - GRASP_BACK, p[1], Z_CARRY]], [0.0], 3.0, f"lift{P}")

    def place(self, P):
        p = POTS[P]
        s, phi = SPOTS[P]
        start = np.array([p[0] - GRASP_BACK, p[1], Z_CARRY])
        end = np.array([s[0] - GRASP_BACK * math.cos(phi), s[1] - GRASP_BACK * math.sin(phi), Z_CARRY])
        tcps, phis = [], []
        for t in np.linspace(0, 1, 5)[1:]:
            tcps.append(start * (1 - t) + end * t); phis.append(phi * t)
        self.go(tcps, phis, 6.0, f"place{P}: transport")
        self.go([[end[0], end[1], Z_RELEASE]], [phi], 3.0, f"place{P}: lower")
        self.gripper(0.04)
        self.obstacles["stove" + P] = np.array([*s, 1.09])
        back = end - 0.06 * np.array([math.cos(phi), math.sin(phi), 0])
        self.go([[back[0], back[1], Z_RELEASE]], [phi], 2.0, f"place{P}: retreat")
        self.go([[back[0], back[1], 1.15]], [phi], 2.0, f"place{P}: rise")

if __name__ == "__main__":
    dry = "--dry" in sys.argv
    phases = [a for a in sys.argv[1:] if not a.startswith("--")]
    t = Task(dry)
    if phases and phases[0] == "goto":   # goto x y z phi_deg seconds
        x, y, z, phi, sec = map(float, phases[1:6])
        t.go([[x, y, z]], [math.radians(phi)], sec, "goto")
        phases = []
    if phases and phases[0] == "grip":
        t.gripper(float(phases[1])); phases = []
    for ph in phases:
        name, P = ph[:-1], ph[-1]
        if ph == "home":
            t.home()
        else:
            getattr(t, name)(P)
    if not dry:
        print("fingers:", t.r.fingers())
