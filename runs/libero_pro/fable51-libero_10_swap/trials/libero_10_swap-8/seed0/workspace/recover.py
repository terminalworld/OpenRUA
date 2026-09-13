#!/usr/bin/env python3
"""Recover a moka pot lying on its side: top-down grasp at the waist, lift, reorient to
vertical (compensating the measured in-hand tilt), place with a no-drag release.
Usage:
  python3 recover.py grasp   wx wy wz  dx dy        # waist axis point (world) + horizontal axis dir (bottom->lid)
  python3 recover.py lift
  python3 recover.py place   theta_deg sx sy phi_deg # theta: pot up-dir tilt in hand x-z plane (lid-up positive)
  python3 recover.py open | home
"""
import math, sys, time
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import kin
from kin import fk_all, fk_hand, ik
from rob import R_to_quat, quat_to_R
import task
from task import Task, TCP_OFF, HOME, R_POS, R_yaw

Z_BOTTOM_TO_WAIST = 0.058     # pot bottom -> grasp point (measured after lift)
PLATE_TOP = 0.930             # burner ring top
GRASP_UP = 0.01               # TCP sits 1 cm off the pot axis, away from the pot (palm side)


def opt(name, default, conv=float):
    for a in sys.argv:
        if a.startswith(f"--{name}="):
            v = a.split("=", 1)[1]
            return [conv(x) for x in v.split(",")] if "," in v else conv(v)
    return default


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


class Recover(Task):
    handle_world = None
    stage = (0.09, 0.13)
    def go_R(self, tcps, Rs, seconds, label="", retry=True):
        seed = self.cur(); qs = []
        for tcp, R in zip(tcps, Rs):
            pos = np.asarray(tcp) - TCP_OFF * R[:, 2]
            q = ik(pos, R_to_quat(R), seed, w_post=0.03, tries=12)
            if q is None:
                raise RuntimeError(f"IK failed for tcp={np.round(tcp,3)}")
            qs.append(q); seed = q
        n = len(qs)
        print(f"[{label}] {n} waypoints, {seconds}s", flush=True)
        for q, tcp in zip(qs, tcps):
            Ts = fk_all(q); tip = Ts[-1][:3, 3] + Ts[-1][:3, 2] * TCP_OFF
            print(f"   q={np.round(q, 3)} tcp={np.round(tip, 3)} minlinkz={min(T[2, 3] for T in Ts[2:]):.3f}", flush=True)
        if self.dry:
            self.q = qs[-1]; return
        dq = max(np.abs(np.diff(np.vstack([self.cur()] + qs), axis=0)).max(axis=1).sum(), 1e-3)
        scale = max(1.0, 2.0 * dq / seconds)
        times = [seconds * (i + 1) / n * scale for i in range(n)]
        code, err = self.r.move_joints(qs, times)
        for k in range(5 if retry else 0):
            if code == 0 and err < 0.01:
                break
            print(f"   retrying final waypoint (code={code}, err={err:.4f})", flush=True)
            code, err = self.r.move_joints([qs[-1]], max(2.0, 2.0 * err))
        pos, quat = self.r.fk(); R = quat_to_R(quat); tip = pos + R[:, 2] * TCP_OFF
        print(f"   TCP now {np.round(tip, 4)} (target {np.round(tcps[-1], 4)}) err={np.linalg.norm(tip - tcps[-1]):.4f}", flush=True)
        return tip

    def open_full(self):
        """The sim clock only advances while a command runs: re-issue open until fully open."""
        for k in range(6):
            f = self.gripper(0.04)
            if self.dry or (f[0] > 0.039 and -f[1] > 0.039):
                break
            print("   fingers not fully open yet, re-issuing", flush=True)

    def hand_R(self):
        if self.dry:
            return fk_hand(self.q)[:3, :3]
        pos, quat = self.r.fk(); return quat_to_R(quat)

    def grasp(self, waist, d0, beta=0.0, sign=-1, pitch=0.0):
        """Top-down grasp of a lying pot. d0: horizontal bottom->lid dir. beta: hand tilt about the
        pot axis (negative leans the hand toward the robot). sign=-1: hand x = -d0 ; +1: hand x = +d0."""
        d0 = np.asarray(d0, float); d0 /= np.linalg.norm(d0); d3 = np.array([d0[0], d0[1], 0.0])
        x = sign * d3; z = np.array([0, 0, -1.0]); y = np.cross(z, x)
        R = Rotation.from_rotvec(beta * d3).as_matrix() @ np.column_stack([x, y, z]) @ Ry(pitch)  # pitch: about finger axis
        print("   grasp hand z:", np.round(R[:, 2], 3), "hand y (fingers):", np.round(R[:, 1], 3))
        tcp = np.asarray(waist) - GRASP_UP * R[:, 2]
        self.gripper(0.04)
        self.go_R([tcp - 0.10 * R[:, 2]], [R], 4.0, "grasp: above")
        self.go_R([tcp], [R], 3.0, "grasp: descend", retry=False)
        return self.gripper(0.0)

    def grasp_R(self, waist, a, gam, sgn, pitch, close=True):
        """Pinch of a lying pot with a general orientation (feas.py parametrization).
        a: horizontal pot axis dir (bottom->lid). Fingers along y = sgn*(cos(gam)*h + sin(gam)*Z), h = a x Z;
        hand z = tilt of the pinch normal by `pitch` toward a (negative: hand leans from the lid side)."""
        a = np.array([a[0], a[1], 0.0]); a /= np.linalg.norm(a)
        Z = np.array([0, 0, 1.0]); h = np.cross(a, Z)
        y = sgn * (math.cos(gam) * h + math.sin(gam) * Z)
        zd = -Z - (-Z @ y) * y; zd /= np.linalg.norm(zd)
        z = math.cos(pitch) * zd + math.sin(pitch) * a
        x = np.cross(y, z)
        R = np.column_stack([x, y, z])
        print("   grasp hand z:", np.round(z, 3), "hand y (fingers):", np.round(y, 3), flush=True)
        tcp = np.asarray(waist) - GRASP_UP * z
        pts = [tcp + s * y + t * z + u * x for s in (-0.051, 0.051) for t in (0.0, 0.01) for u in (-0.009, 0.009)]
        print("   lowest finger corner:", np.round(min(pts, key=lambda v: v[2]), 3), flush=True)
        self.open_full()
        self.go_R([tcp - 0.10 * z], [R], 4.0, "grasp: above")
        self.go_R([tcp], [R], 3.0, "grasp: descend", retry=False)
        return self.gripper(0.0) if close else None

    def place_auto(self, a_world, spot, phi):
        """Place using the pot up-direction measured in world (assumed rigid in the hand since grasp)."""
        R0 = self.hand_R()
        a = np.array([a_world[0], a_world[1], a_world[2] if len(a_world) > 2 else 0.0]); a /= np.linalg.norm(a)
        up_hand = R0.T @ a
        theta = math.atan2(-up_hand[2], -up_hand[0])
        print(f"   pot up in hand frame {np.round(up_hand,3)} -> theta={math.degrees(theta):.1f} deg", flush=True)
        if abs(up_hand[1]) > 0.1:
            print("   !! pot axis not in hand x-z plane; placement will be tilted", flush=True)
        return self.place(theta, spot, phi)

    def relocate(self, new_xy, z_g, z_carry=1.05):
        """Carry the (lying) pot with the current hand orientation to new_xy, set it down with the TCP at z_g, release, back off."""
        R = self.hand_R()
        pos = fk_hand(self.q)[:3, 3] if self.dry else self.r.fk()[0]
        tip0 = pos + R[:, 2] * TCP_OFF
        self.go_R([[tip0[0], tip0[1], z_carry]], [R], 3.0, "relocate: lift")
        self.go_R([[new_xy[0], new_xy[1], z_carry]], [R], 4.0, "relocate: transport")
        self.go_R([[new_xy[0], new_xy[1], z_g + 0.004]], [R], 3.0, "relocate: lower", retry=False)
        self.open_full()
        back = np.array([new_xy[0], new_xy[1], z_g + 0.004]) - 0.09 * R[:, 2]
        self.go_R([back], [R], 3.0, "relocate: retreat")
        self.go_R([[back[0], back[1], back[2] + 0.12]], [R], 2.0, "relocate: rise")

    def lift(self, z=1.10):
        R = self.hand_R()
        pos, quat = (fk_hand(self.q)[:3, 3], None) if self.dry else self.r.fk()
        tip = pos + R[:, 2] * TCP_OFF
        self.go_R([[tip[0], tip[1], z]], [R], 3.0, "lift")

    def place(self, theta, spot, phi):
        """theta: tilt of pot up-direction in the hand x-z plane; up = -cos(th) x - sin(th) z (hand frame)."""
        R0 = self.hand_R()
        pos = fk_hand(self.q)[:3, 3] if self.dry else self.r.fk()[0]
        tip0 = pos + R0[:, 2] * TCP_OFF
        Rf = R_yaw(phi) @ R_POS @ Ry(theta)
        if Rf[:, 2] @ np.array([math.cos(phi), math.sin(phi), 0]) < 0:   # keep approach azimuth = phi
            Rf = R_yaw(phi + math.pi) @ R_POS @ Ry(theta)
        if self.handle_world is not None:
            print("   handle dir now:", np.round(self.handle_world, 2), "-> final:", np.round(Rf @ R0.T @ self.handle_world, 2))
        # check: pot up in world
        up_hand = np.array([-math.cos(theta), 0, -math.sin(theta)])
        print("   final pot up (world):", np.round(Rf @ up_hand, 3), " final hand z:", np.round(Rf[:, 2], 3))
        waist_z = PLATE_TOP + Z_BOTTOM_TO_WAIST
        waist_f = np.array([spot[0], spot[1], waist_z])
        tcp_f = waist_f - GRASP_UP * Rf[:, 2]
        tcp_high = tcp_f.copy(); tcp_high[2] = tip0[2]
        stage = np.array([self.stage[0], self.stage[1], tip0[2]])
        # 1) translate closer to the robot (better dexterity), 2) slerp orientation there, 3) translate above spot
        sl = Slerp([0, 1], Rotation.from_matrix([R0, Rf]))
        ts = np.linspace(0, 1, 6)[1:]
        tcps = [tip0 * (1 - t) + stage * t for t in (0.5, 1.0)] + [stage] * 5 + [stage * (1 - t) + tcp_high * t for t in (0.5, 1.0)]
        Rs = [R0, R0] + [sl(t).as_matrix() for t in ts] + [Rf, Rf]
        for t, R in zip(ts, Rs[2:7]):
            up = R @ up_hand
            print(f"   t={t:.1f} pot up={np.round(up,2)} lowest pot pt z~{(tip0[2]-0.065*max(up[2],0)-0.09*max(-up[2],0)-0.04):.3f}")
        self.go_R(tcps, Rs, 12.0, "place: stage+reorient+transport")
        # lower to 1.5 cm above the nominal touchdown, then creep down until the arm can no longer
        # follow (pot bottom on the plate) or a 1 cm floor below nominal is reached
        low = tcp_f + [0, 0, 0.015]
        self.go_R([low], [Rf], 4.0, "place: lower")
        tip = low.copy()
        while low[2] > tcp_f[2] - 0.010 + 1e-6:
            low = low - [0, 0, 0.004]
            tip = self.go_R([low], [Rf], 1.0, "place: creep", retry=False)
            if not self.dry and tip[2] > low[2] + 0.0025:
                print(f"   touchdown detected: TCP z {tip[2]:.4f} vs cmd {low[2]:.4f}", flush=True)
                break
        if self.dry:
            tip = low
        self.open_full()
        back = np.array([math.cos(phi), math.sin(phi), 0.0])
        rel = np.array([tip[0], tip[1], tip[2]])
        self.go_R([rel - 0.03 * back], [Rf], 2.0, "place: retreat 1")
        self.go_R([rel - 0.07 * back], [Rf], 2.0, "place: retreat 2")
        self.go_R([rel - 0.07 * back + [0, 0, 0.15]], [Rf], 2.0, "place: rise")


if __name__ == "__main__":
    dry = "--dry" in sys.argv
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    Z_BOTTOM_TO_WAIST = opt("zbw", Z_BOTTOM_TO_WAIST)
    GRASP_UP = opt("up", GRASP_UP)
    t = Recover(dry)
    st = opt("stage", None)
    if st is not None:
        t.stage = tuple(st)
    hw = opt("handle", None)
    if hw is not None:
        t.handle_world = np.array([hw[0], hw[1], 0.0])
    if a[0] == "grasp":   # grasp wx wy wz dx dy [beta_deg] [sign]
        t.grasp([float(a[1]), float(a[2]), float(a[3])], [float(a[4]), float(a[5])],
                math.radians(float(a[6])) if len(a) > 6 else 0.0, int(a[7]) if len(a) > 7 else -1,
                math.radians(float(a[8])) if len(a) > 8 else 0.0)
    elif a[0] == "graspR":   # graspR wx wy wz ax ay gamma_deg sgn pitch_deg
        t.grasp_R([float(a[1]), float(a[2]), float(a[3])], [float(a[4]), float(a[5])],
                  math.radians(float(a[6])), int(a[7]), math.radians(float(a[8])))
    elif a[0] == "placeauto":   # placeauto ax ay az sx sy phi_deg   (a: pot bottom->lid dir in world, as at grasp)
        t.place_auto([float(a[1]), float(a[2]), float(a[3])], (float(a[4]), float(a[5])), math.radians(float(a[6])))
    elif a[0] == "relocate":   # relocate nx ny z_tcp
        t.relocate((float(a[1]), float(a[2])), float(a[3]))
    elif a[0] == "lift":
        t.lift(float(a[1]) if len(a) > 1 else 1.10)
    elif a[0] == "place":
        t.place(math.radians(float(a[1])), (float(a[2]), float(a[3])), math.radians(float(a[4])))
    elif a[0] == "open":
        t.gripper(0.04)
    elif a[0] == "home":
        t.home()
    if not dry:
        print("fingers:", t.r.fingers())
