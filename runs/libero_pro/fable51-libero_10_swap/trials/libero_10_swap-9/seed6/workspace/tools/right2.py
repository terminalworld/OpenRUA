#!/usr/bin/env python3
"""Second righting attempt. Mug lies along x (rim at +x end), axis (y=AX_Y, z=AX_Z),
handle up-and--y at HANDLE_ANG deg from +y (measured in the y-z plane).
  python3 tools/right2.py roll      # push the handle over the top so it points up (+y roll)
  python3 tools/right2.py pinch     # top-down pinch of the handle bar
  python3 tools/right2.py yaw       # lift, rotate so the mouth faces -x, move to SET_X, set down
  python3 tools/right2.py scoop     # closed fingers into the mouth from -x, arc to tip it upright
"""
import sys
import numpy as np
import rclpy
sys.path.insert(0, "/workspace/tools")
from robot import mat_to_quat
from planner import Planner

AX_Y, AX_Z = -0.50, 0.953
BAR_X = -0.17                    # mid of the handle bar along x
HANDLE_ANG = np.deg2rad(130.0)   # bar direction from axis, angle from +y toward +z
R_MUG, R_BAR = 0.0485, 0.0785
R_TD = np.array([[-1.0, 0, 0], [0, 1.0, 0], [0, 0, -1.0]])   # hand down, fingers close along y
Q_TD = mat_to_quat(R_TD)


def Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def roll_path():
    pts = []
    for th_deg in np.arange(135, 84, -10):
        th = np.deg2rad(th_deg)
        ax_y = AX_Y + R_MUG * (HANDLE_ANG - th)          # mug rolls toward +y
        bar = np.array([BAR_X, ax_y + R_BAR * np.cos(th), AX_Z + R_BAR * np.sin(th)])
        pts.append(bar + np.array([0, -0.0175, 0.0]))
    return pts


def main():
    stage = sys.argv[1]
    rclpy.init(); p = Planner()
    if stage == "roll":
        p.set_scene(microwave="solid", yellow=False)
        p.grip(False)
        pts = roll_path()
        pre = pts[0] + np.array([0, -0.01, 0.06])
        ok = p.goto_tcp(pre, Q_TD)
        print("pre ok", ok)
        ok = p.line(pts[0] + np.array([0, -0.01, 0]), Q_TD, step=0.005, avoid=False, time_scale=3)
        print("down ok", ok)
        for i, pt in enumerate(pts):
            ok = p.line(pt, Q_TD, step=0.004, avoid=False, min_fraction=0.9, time_scale=5)
            print(f"roll {i} target {np.round(pt, 3)} ok {ok} tcp {np.round(p.tcp()[0], 4)}", flush=True)
            if not ok:
                break
        tcp = p.tcp()[0]
        p.line(tcp + np.array([0, -0.01, 0.06]), Q_TD, step=0.01, avoid=False, min_fraction=0.9)
    elif stage == "pinch":
        bar = np.array([float(v) for v in sys.argv[2:5]]) if len(sys.argv) > 4 else None
        p.set_scene(microwave="solid", yellow=False)
        p.grip(True)
        pre = bar + np.array([0, 0, 0.06])
        ok = p.goto_tcp(pre, Q_TD)
        print("pre ok", ok)
        ok = p.line(bar, Q_TD, step=0.004, avoid=False, time_scale=4)
        print("descend ok", ok, "tcp", np.round(p.tcp()[0], 4))
        p.grip(False)
        print("gap", round(p.finger_gap(), 4))
    elif stage == "yaw":
        # args: mug axis direction base->rim angle (deg, from +x), set-down x
        ang_now = np.deg2rad(float(sys.argv[2])); set_x = float(sys.argv[3])
        tcp, _, R = p.tcp()
        p.line(tcp + np.array([0, 0, 0.04]), mat_to_quat(R), step=0.005, avoid=False, time_scale=3)
        print("lift gap", round(p.finger_gap(), 4))
        # rotate about z so base->rim points to -x (angle pi)
        dth = (np.pi - ang_now + np.pi) % (2 * np.pi) - np.pi
        print("rotate by deg", np.rad2deg(dth))
        R2 = Rz(dth) @ R
        target = np.array([set_x, tcp[1], tcp[2] + 0.04])
        ok = p.goto_tcp(target, mat_to_quat(R2))
        print("rotate/move ok", ok, "gap", round(p.finger_gap(), 4))
        ok = p.line(np.array([set_x, tcp[1], tcp[2] + 0.004]), mat_to_quat(R2), step=0.005, avoid=False, time_scale=3)
        print("lower ok", ok)
        p.grip(True)
        t2 = p.tcp()[0]
        p.line(t2 + np.array([0, 0, 0.08]), mat_to_quat(R2), step=0.01, avoid=False, min_fraction=0.9)
    elif stage == "scoop":
        # args: rim x, base x, axis y, axis z
        x_r, x_b, ay, az = map(float, sys.argv[2:6])
        pitch = np.deg2rad(60.0)
        a = np.array([np.cos(pitch), 0, -np.sin(pitch)])
        xh = np.array([0, 1.0, 0]); yh = np.cross(a, xh)
        R = np.stack([xh, yh, a], 1); quat = mat_to_quat(R)
        depth = min(0.02, 0.0345 / np.tan(pitch))
        tcp0 = np.array([x_r + depth, ay, az - 0.01])
        C = np.array([x_b, 0.90])
        v0 = np.array([tcp0[0] - C[0], tcp0[2] - C[1]]); ang0 = np.arctan2(v0[1], v0[0]); rad = np.linalg.norm(v0)
        pts = []
        for th in np.linspace(0, np.pi / 2, 7)[1:]:
            ang = ang0 + th          # -x end rises toward +z: angle increases toward 90deg
            pts.append(np.array([C[0] + rad * np.cos(ang), ay, C[1] + rad * np.sin(ang)]))
        print("tcp0", np.round(tcp0, 4), "arc", [np.round(q, 3).tolist() for q in pts])
        p.set_scene(microwave="solid", yellow=False)
        p.grip(False)
        pre = tcp0 - 0.05 * a
        ok = p.goto_tcp(pre, quat)
        print("pre ok", ok, "tcp", np.round(p.tcp()[0], 4))
        ok = p.line(tcp0, quat, step=0.005, avoid=False, time_scale=4)
        print("insert ok", ok, "tcp", np.round(p.tcp()[0], 4))
        for i, pt in enumerate(pts):
            ok = p.line(pt, quat, step=0.005, avoid=False, min_fraction=0.9, time_scale=5)
            print(f"arc {i} target {np.round(pt, 3)} ok {ok} tcp {np.round(p.tcp()[0], 4)}", flush=True)
            if not ok:
                break
        tcp = p.tcp()[0]
        ok = p.line(tcp + np.array([0, 0, 0.08]), quat, step=0.01, avoid=False, min_fraction=0.9, time_scale=3)
        print("retreat ok", ok, "tcp", np.round(p.tcp()[0], 4))
    p.destroy_node(); rclpy.shutdown()


if __name__ == "__main__" and sys.argv[1] not in ("tilt", "tiltinfo", "flip"):
    main()


def Ry(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])


def tilt_frames(phi_deg, tilt_deg, p0, p1, n=8, R0=R_TD):
    """Keyframes rotating the TD pinch about world y by -tilt (rim +x -> up) and yawing by phi,
    while the TCP slides from p0 to p1."""
    out = []
    for k in range(1, n + 1):
        f = k / n
        R = Rz(np.deg2rad(phi_deg) * f) @ Ry(-np.deg2rad(tilt_deg) * f) @ R0
        out.append((p0 + (p1 - p0) * f, mat_to_quat(R)))
    return out


def tilt_main(stage):
    rclpy.init(); p = Planner()
    if stage == "flip":
        # release, lift, rotate the hand by pi about its z (joint 7), re-pinch at the same TCP
        tcp, quat, R = p.tcp()
        p.grip(True)
        p.line(tcp + np.array([0, 0, 0.06]), quat, step=0.01, avoid=False, min_fraction=0.9)
        q = p.arm_q(); q[6] += np.pi if q[6] < 0 else -np.pi
        p.move(q, 25.0)
        tcp2, quat2, R2 = p.tcp()
        print("flipped R\n", np.round(R2, 3))
        ok = p.line(tcp, quat2, step=0.004, avoid=False, time_scale=4)
        print("descend ok", ok, "tcp", np.round(p.tcp()[0], 4))
        p.grip(False)
        print("gap", round(p.finger_gap(), 4))
        p.destroy_node(); rclpy.shutdown(); return
    phi, tilt = float(sys.argv[2]), float(sys.argv[3])
    p1 = np.array([float(v) for v in sys.argv[4:7]])        # TCP at end of rotation (z high)
    z_set = float(sys.argv[7])
    tcp, _, R = p.tcp()
    p0 = np.array([tcp[0], tcp[1], p1[2]])
    frames = tilt_frames(phi, tilt, p0, p1, R0=R)
    h = Rz(np.deg2rad(phi)) @ Ry(-np.deg2rad(tilt)) @ np.array([0, 0, 1.0])
    print("handle dir at end", np.round(h, 3), "mug centre offset", np.round(-0.0785 * h, 4))
    if stage == "tiltinfo":
        seed = None
        for pos, q in frames + [(np.array([p1[0], p1[1], z_set]), frames[-1][1])]:
            s = p.ik(pos, q, seed=seed, attempts=4)
            print(np.round(pos, 3), "ok" if s is not None else "FAIL")
            if s is not None:
                seed = s
        return
    ok = p.line(p0, mat_to_quat(R), step=0.005, avoid=False, time_scale=3)
    print("lift ok", ok, "gap", round(p.finger_gap(), 4))
    jt, frac = p.cartesian_poses(frames, step=0.005)
    print("cartesian fraction", frac)
    if jt is None or frac < 0.95:
        return
    p.execute(jt, time_scale=6)
    print("rotated; gap", round(p.finger_gap(), 4), "tcp", np.round(p.tcp()[0], 4))
    q_end = frames[-1][1]
    ok = p.line(np.array([p1[0], p1[1], z_set]), q_end, step=0.005, avoid=False, time_scale=4)
    print("lowered ok", ok, "gap", round(p.finger_gap(), 4))
    p.grip(True)
    a = Rz(np.deg2rad(phi)) @ Ry(-np.deg2rad(tilt)) @ np.array([0, 0, -1.0])
    t2 = p.tcp()[0]
    p.line(t2 - 0.05 * a + np.array([0, 0, 0.05]), q_end, step=0.01, avoid=False, min_fraction=0.9)
    print("retreated; tcp", np.round(p.tcp()[0], 4))
    p.destroy_node(); rclpy.shutdown()


if __name__ == "__main__" and sys.argv[1] in ("tilt", "tiltinfo", "flip"):
    tilt_main(sys.argv[1])
