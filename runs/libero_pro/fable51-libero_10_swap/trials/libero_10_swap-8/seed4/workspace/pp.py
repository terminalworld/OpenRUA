"""Pick-and-place primitives for the moka pots (tilted side grasp)."""
import sys, time
import numpy as np
from robot import *

TILT = np.radians(20)          # hand z-axis below horizontal, pointing +x
GRASP_Z = 0.938                # TCP height for the lower-chamber grasp
TABLE_Z = 0.90


def R_tilt(tilt=TILT):
    a = -(np.pi / 2 - tilt)
    return rot_y(a) @ rot_x(np.pi)


R_G = R_tilt()
ZH = R_G[:, 2]                  # hand approach axis in world


def hand_from_tcp(tcp):
    return np.asarray(tcp) - TCP * ZH


class PP:
    def __init__(self):
        self.r = Robot()

    def ik_path(self, tcps, seed=None, max_jump=0.8):
        """IK for a list of TCP positions (orientation R_G); seeds chain."""
        q = self.r.arm_q() if seed is None else seed
        out = []
        for p in tcps:
            s = self.r.ik_tcp(p, R_G, seed=q, timeout=3)
            if s is None:
                raise RuntimeError(f'IK failed at TCP {np.round(p, 3)}')
            jump = np.abs(s - q).max()
            if jump > max_jump and out:
                # try again seeded from previous once more
                s2 = self.r.ik_tcp(p, R_G, seed=q, timeout=3)
                if s2 is not None and np.abs(s2 - q).max() < jump:
                    s = s2
                    jump = np.abs(s - q).max()
                if jump > max_jump:
                    print(f'  warn: joint jump {jump:.2f} at {np.round(p,3)}')
            out.append(s)
            q = s
        return out

    def exec(self, qs, total_t, label=''):
        n = len(qs)
        ts = [total_t * (i + 1) / n for i in range(n)]
        code = self.r.move(qs, ts)
        err = np.abs(self.r.arm_q() - qs[-1]).max()
        print(f'  [{label}] code={code} final joint err={err:.4f}')
        if code != 0 or err > 0.05:
            print('  retrying last point')
            code = self.r.move([qs[-1]], [2.0])
            err = np.abs(self.r.arm_q() - qs[-1]).max()
            print(f'  [{label}] retry code={code} err={err:.4f}')
        return code, err

    def line(self, p0, p1, n=None):
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        d = np.linalg.norm(p1 - p0)
        if n is None:
            n = max(2, int(np.ceil(d / 0.03)))
        return [p0 + (p1 - p0) * (i + 1) / n for i in range(n)]

    def tcp_now(self):
        p, R = self.r.fk(self.r.arm_q(), 'panda_hand')
        return p + TCP * R[:, 2], R

    def move_tcp_line(self, p1, t=None, label='line'):
        p0, _ = self.tcp_now()
        pts = self.line(p0, p1)
        qs = self.ik_path(pts)
        if t is None:
            t = max(1.5, np.linalg.norm(np.asarray(p1) - p0) / 0.08)
        return self.exec(qs, t, label)

    def goto_tcp(self, p1, t=3.0, label='goto'):
        qs = self.ik_path([p1])
        return self.exec(qs, t, label)

    def pick(self, center):
        cx, cy = center
        g = np.array([cx, cy, GRASP_Z])
        pre = g - 0.10 * ZH                      # back along approach axis
        high = pre + np.array([0, 0, 0.12])
        print('open gripper', self.r.gripper(0.04))
        print('-> high pre-grasp', np.round(high, 3))
        self.goto_tcp(high, 4.0, 'high')
        print('-> pre-grasp', np.round(pre, 3))
        self.move_tcp_line(pre, 2.5, 'pre')
        print('-> grasp', np.round(g, 3))
        self.move_tcp_line(g, 3.0, 'approach')
        p, _ = self.tcp_now(); print('  tcp at', np.round(p, 4))
        res = self.r.gripper(0.0)
        gap = self.r.finger_gap()
        print(f'close gripper {res} gap={gap:.4f}')
        return gap

    def lift(self, dz=0.12):
        p, _ = self.tcp_now()
        self.move_tcp_line(p + np.array([0, 0, dz]), 3.0, 'lift')

    def place(self, target, z_bottom=0.935, carry_z=1.06):
        tx, ty = target
        rel_z = z_bottom + (GRASP_Z - TABLE_Z)     # TCP z when pot bottom at z_bottom
        above = np.array([tx, ty, carry_z])
        down = np.array([tx, ty, rel_z])
        p0, _ = self.tcp_now()
        print('-> carry to above target', np.round(above, 3))
        pts = self.line(p0, above, n=6)
        qs = self.ik_path(pts)
        self.exec(qs, 6.0, 'carry')
        print('-> lower', np.round(down, 3))
        self.move_tcp_line(down, 3.0, 'lower')
        print('open gripper', self.r.gripper(0.04))
        gap = self.r.finger_gap(); print(f'  gap={gap:.4f}')
        back = down - 0.10 * ZH
        print('-> retreat', np.round(back, 3))
        self.move_tcp_line(back, 2.5, 'retreat')
        self.move_tcp_line(back + np.array([0, 0, 0.10]), 2.0, 'up')
