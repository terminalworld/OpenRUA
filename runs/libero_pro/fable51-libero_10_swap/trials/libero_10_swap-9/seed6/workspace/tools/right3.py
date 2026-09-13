#!/usr/bin/env python3
"""Third righting attempt.
  python3 tools/right3.py pinchinfo bx by bz bar_ang_deg      # IK/joint-7 check for TD pinch + 120deg yaw
  python3 tools/right3.py pinch bx by bz bar_ang_deg yaw_deg  # TD pinch of the handle top bar
  python3 tools/right3.py carry dyaw_deg x y                  # lift, yaw the lying mug, set it down (bar centre at x,y)
  python3 tools/right3.py scoopinfo x_rim y_axis z_axis [pitch]
  python3 tools/right3.py scoop x_rim y_axis z_axis [pitch]   # closed fingers into the mouth from -x, arc about base edge
"""
import sys
import numpy as np
import rclpy
sys.path.insert(0, "/workspace/tools")
from robot import mat_to_quat
from planner import Planner

R_TD = np.array([[-1.0, 0, 0], [0, 1.0, 0], [0, 0, -1.0]])   # hand down, fingers close along y
MUG_LEN, R_RIM = 0.105, 0.0485
BAR_Z = 1.026                                                  # TCP height for pinching the top bar


def Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def td_R(bar_ang_deg, flip=False):
    """TD hand with the closing axis perpendicular to a bar at bar_ang (deg from +x)."""
    # R_TD has hand y = +Y (closing axis); yaw so the closing axis is perpendicular to the bar
    return Rz(np.deg2rad(bar_ang_deg) + (np.pi if flip else 0.0)) @ R_TD


def scoop_frame(pitch_deg, flip=True):
    pth = np.deg2rad(pitch_deg)
    a = np.array([np.cos(pth), 0, -np.sin(pth)])      # approach: toward +x and down
    yh = np.array([0, -1.0 if flip else 1.0, 0])      # flip: wrist offset toward +x (away from the door)
    xh = np.cross(yh, a)
    return a, mat_to_quat(np.stack([xh, yh, a], 1))


def scoop_arc(x_rim, y_ax, z_ax, pitch_deg, n=9, dz=-0.01):
    depth = min(0.02, 0.0345 / np.tan(np.deg2rad(pitch_deg)))
    tcp0 = np.array([x_rim + depth, y_ax, z_ax + dz])
    C = np.array([x_rim + MUG_LEN, 0.90])               # base edge on the table (x, z)
    v0 = np.array([tcp0[0] - C[0], tcp0[2] - C[1]])
    ang0, rad = np.arctan2(v0[1], v0[0]), np.linalg.norm(v0)
    pts = []
    for th in np.linspace(0, np.pi / 2, n + 1)[1:]:
        ang = ang0 - th                                  # rim end rises and moves toward +x
        pts.append(np.array([C[0] + rad * np.cos(ang), y_ax, C[1] + rad * np.sin(ang)]))
    return tcp0, pts


def pick_branch(p, pos, quat, tries=40):
    """IK solution whose wrist (link5) sits on the +y side of the hand and elbow (link4) is high."""
    from robot import LIMITS
    rng = np.random.default_rng(0)
    best = None
    for _ in range(tries):
        seed = np.array([rng.uniform(lo + 0.3, hi - 0.3) for lo, hi in LIMITS])
        s = p.ik(pos, quat, seed=seed, attempts=1)
        if s is None:
            continue
        l4 = p.fk(s, "panda_link4")[0]; l5 = p.fk(s, "panda_link5")[0]; l7 = p.fk(s, "panda_link7")[0]
        d = (l5 - l7) / 0.088
        if d[1] > 0.9 and l4[2] > 1.30:
            print("branch", np.round(s, 2), "d", np.round(d, 2), "l4", np.round(l4, 3), "l5", np.round(l5, 3))
            if best is None or l4[2] > best[1]:
                best = (s, l4[2])
    return None if best is None else best[0]


def main():
    stage = sys.argv[1]
    rclpy.init(); p = Planner()
    p.set_scene(microwave="solid", yellow=False)
    if stage == "pinchinfo":
        bx, by, bz, ang = map(float, sys.argv[2:6])
        for flip in (False, True):
            R0 = td_R(ang, flip)
            seed = None
            print("flip", flip)
            for k in range(0, 7):
                dy = np.deg2rad(120.0) * k / 6
                pos = np.array([bx, by, bz + (0.04 if k else 0)])
                s = p.ik(pos, mat_to_quat(Rz(dy) @ R0), seed=seed, attempts=3)
                print(f"  yaw+{np.rad2deg(dy):5.1f}", None if s is None else np.round(s, 2))
                if s is not None:
                    seed = s
    elif stage == "pinch":
        bx, by, bz, ang = map(float, sys.argv[2:6]); flip = len(sys.argv) > 6 and sys.argv[6] == "flip"
        quat = mat_to_quat(td_R(ang, flip))
        p.grip(True)
        bar = np.array([bx, by, bz])
        ok = p.goto_tcp(bar + np.array([0, 0, 0.06]), quat)
        print("pre ok", ok, "q", np.round(p.arm_q(), 2))
        ok = p.line(bar, quat, step=0.004, avoid=False, time_scale=4)
        print("descend ok", ok, "tcp", np.round(p.tcp()[0], 4))
        p.grip(False)
        print("gap", round(p.finger_gap(), 4))
    elif stage == "carry":
        dyaw = np.deg2rad(float(sys.argv[2])); tx, ty = float(sys.argv[3]), float(sys.argv[4])
        tcp, quat, R = p.tcp()
        ok = p.line(tcp + np.array([0, 0, 0.04]), quat, step=0.005, avoid=False, time_scale=3)
        print("lift ok", ok, "gap", round(p.finger_gap(), 4))
        # yaw + translate in keyframes
        frames = []
        n = 8
        p0 = tcp + np.array([0, 0, 0.04]); p1 = np.array([tx, ty, p0[2]])
        for k in range(1, n + 1):
            f = k / n
            frames.append((p0 + (p1 - p0) * f, mat_to_quat(Rz(dyaw * f) @ R)))
        jt, frac = p.cartesian_poses(frames, step=0.005)
        print("cartesian fraction", frac)
        if jt is None or frac < 0.95:
            p.destroy_node(); rclpy.shutdown(); return
        p.execute(jt, time_scale=6)
        q_end = frames[-1][1]
        print("yawed; gap", round(p.finger_gap(), 4), "tcp", np.round(p.tcp()[0], 4))
        ok = p.line(np.array([tx, ty, tcp[2]]), q_end, step=0.005, avoid=False, time_scale=4)
        print("lowered ok", ok, "gap", round(p.finger_gap(), 4), "tcp", np.round(p.tcp()[0], 4))
        p.grip(True)
        t2 = p.tcp()[0]
        p.line(t2 + np.array([0, 0, 0.08]), q_end, step=0.01, avoid=False, min_fraction=0.9)
        print("retreated; tcp", np.round(p.tcp()[0], 4))
    elif stage == "hinge":
        # TD pinch of the top bar acts as a friction hinge (weak axis = world y): move the TCP along
        # an arc about the base edge on the table so the mug pivots upright.  args: x_pivot sign(+1/-1)
        x_piv, sgn = float(sys.argv[2]), float(sys.argv[3])       # sgn -1: rim at +x, mug tips toward -x
        tcp, quat, R = p.tcp()
        C = np.array([x_piv, 0.90])
        v0 = np.array([tcp[0] - C[0], tcp[2] - C[1]]); ang0 = np.arctan2(v0[1], v0[0]); rad = np.linalg.norm(v0)
        print("arc radius", round(rad, 4), "ang0", round(np.rad2deg(ang0), 1))
        for k in range(1, 10):
            ang = ang0 - sgn * np.deg2rad(10.0 * k)
            pt = np.array([C[0] + rad * np.cos(ang), tcp[1], C[1] + rad * np.sin(ang)])
            ok = p.line(pt, quat, step=0.004, avoid=False, min_fraction=0.9, time_scale=5)
            print(f"arc {k} target {np.round(pt, 3)} ok {ok} tcp {np.round(p.tcp()[0], 4)} gap {p.finger_gap():.4f}", flush=True)
            if not ok:
                break
        p.grip(True)
        t2 = p.tcp()[0]
        p.line(t2 + np.array([0, 0, 0.10]), quat, step=0.01, avoid=False, min_fraction=0.9)
        print("retreated; tcp", np.round(p.tcp()[0], 4))
    elif stage == "hinge2":
        # like hinge, but the commanded radius is shortened by delta so the hand presses the mug
        # toward the pivot (base edge into the table) instead of dragging it.  args: x_pivot sgn delta
        x_piv, sgn, delta = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
        step_deg = float(sys.argv[5]) if len(sys.argv) > 5 else 10.0
        tcp, quat, R = p.tcp()
        C = np.array([x_piv, 0.90])
        v0 = np.array([tcp[0] - C[0], tcp[2] - C[1]]); ang0 = np.arctan2(v0[1], v0[0]); rad = np.linalg.norm(v0)
        print("arc radius", round(rad, 4), "ang0", round(np.rad2deg(ang0), 1), "delta", delta)
        lag = 0
        k = 1
        while np.rad2deg(ang0) - sgn * step_deg * k <= 180.0 + 1e-6 and k * step_deg <= 90.0 + 1e-6:
            ang = ang0 - sgn * np.deg2rad(step_deg * k)
            pt = np.array([C[0] + (rad - delta) * np.cos(ang), tcp[1], C[1] + (rad - delta) * np.sin(ang)])
            ok = p.line(pt, quat, step=0.004, avoid=False, min_fraction=0.9, time_scale=5)
            now = p.tcp()[0]
            err = np.linalg.norm(now - pt)
            print(f"arc {k} target {np.round(pt, 3)} ok {ok} tcp {np.round(now, 4)} err {err:.4f} gap {p.finger_gap():.4f}", flush=True)
            lag = lag + 1 if err > 0.015 else 0
            if lag >= 2:
                print("stalled"); break
            k += 1
        p.grip(True)
        t2 = p.tcp()[0]
        p.line(t2 + np.array([0, 0, 0.10]), quat, step=0.01, avoid=False, min_fraction=0.9)
        print("retreated; tcp", np.round(p.tcp()[0], 4))
    elif stage in ("scoopinfo", "scoop"):
        x_r, y_a, z_a = map(float, sys.argv[2:5])
        pitch = float(sys.argv[5]) if len(sys.argv) > 5 else 60.0
        dz = float(sys.argv[6]) if len(sys.argv) > 6 else -0.01
        a, quat = scoop_frame(pitch)
        tcp0, pts = scoop_arc(x_r, y_a, z_a, pitch, dz=dz)
        pre = tcp0 - 0.05 * a
        print("pre", np.round(pre, 4), "tcp0", np.round(tcp0, 4))
        if stage == "scoopinfo":
            seed = None
            for q_ in [pre, tcp0] + pts:
                s = p.ik(q_, quat, seed=seed, attempts=4)
                print(np.round(q_, 3), "FAIL" if s is None else np.round(s, 2))
                if s is not None:
                    seed = s
        else:
            p.grip(False)
            q_pre = pick_branch(p, pre, quat)
            if q_pre is None:
                print("no suitable IK branch"); p.destroy_node(); rclpy.shutdown(); return
            ok = p.goto_q(q_pre, time_s=8.0)
            print("pre ok", ok, "tcp", np.round(p.tcp()[0], 4), "q", np.round(p.arm_q(), 2))
            if not ok:
                p.destroy_node(); rclpy.shutdown(); return
            ok = p.line(tcp0, quat, step=0.005, avoid=False, time_scale=4)
            print("insert ok", ok, "tcp", np.round(p.tcp()[0], 4))
            for i, pt in enumerate(pts):
                ok = p.line(pt, quat, step=0.005, avoid=False, min_fraction=0.9, time_scale=5)
                print(f"arc {i} target {np.round(pt, 3)} ok {ok} tcp {np.round(p.tcp()[0], 4)}", flush=True)
                if not ok:
                    break
            tcp = p.tcp()[0]
            ok = p.line(tcp + np.array([0, 0, 0.10]), quat, step=0.01, avoid=False, min_fraction=0.9, time_scale=3)
            print("retreat ok", ok, "tcp", np.round(p.tcp()[0], 4))
    p.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
