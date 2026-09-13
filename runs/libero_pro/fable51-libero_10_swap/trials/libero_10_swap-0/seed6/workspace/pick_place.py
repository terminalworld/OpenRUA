#!/usr/bin/env python3
"""Pick a can (top-down grasp) and drop it into the basket.
Usage: python3 -u pick_place.py <x> <y> <top_z> <name>
All coordinates world frame (table top z=0.42)."""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from rob import Robot, Q_DOWN


def q_tilt(theta, yaw=0.0):
    """Hand approach tilted by theta from vertical; the wrist is displaced
    from the fingertips along the xy direction given by yaw (0 = +x)."""
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, 0, -s], [0, -1, 0], [-s, 0, -c]])
    R = Rot.from_euler('z', yaw).as_matrix() @ R
    return tuple(Rot.from_matrix(R).as_quat())

BASKET = np.array([0.0, 0.26])
RIM_Z = 0.627


def log(*a):
    print(*a, flush=True)


def cart_path(r, start_q, pts, quat=Q_DOWN):
    """IK for a chain of TCP points, each seeded with the previous one."""
    qs, seed = [], start_q
    for p in pts:
        q = r.ik_tcp_world(p, quat, seed=seed)
        if q is None:
            raise SystemExit(f"IK failed at {p}")
        qs.append(q)
        seed = q
    return qs


def go(r, qs, seconds, tol=0.02):
    code, err = r.move(qs[-1], seconds, via=qs[:-1])
    tcp = r.tcp_world()
    log(f"  move code={code} joint_err={err:.4f} tcp={tcp.round(3)}")
    # controller lag shows up as tolerance violations: resend the final
    # point until the joints actually converge
    for i in range(4):
        if err <= tol:
            break
        code, err = r.move(qs[-1], max(2.0, seconds / 2))
        tcp = r.tcp_world()
        log(f"  retry{i} code={code} joint_err={err:.4f} tcp={tcp.round(3)}")
    return tcp


def main():
    x, y, top, name = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
    tilt = np.radians(float(sys.argv[5])) if len(sys.argv) > 5 else 0.0
    yaw = np.radians(float(sys.argv[6])) if len(sys.argv) > 6 else 0.0
    QG = q_tilt(tilt, yaw) if tilt else Q_DOWN
    r = Robot("pick_" + name)
    log(f"[{name}] start tcp={r.tcp_world().round(3)} fingers={r.fingers()}")

    grasp_z = 0.42 + (top - 0.42) * 0.45      # a bit below mid-height
    pre_z = top + 0.10
    lift_z = 0.76
    log(f"[{name}] target ({x},{y}) top={top} grasp_z={grasp_z:.3f}")

    log("open gripper"); log(" ", r.gripper(0.04))

    q0 = r.joints()
    pre = cart_path(r, q0, [(x, y, lift_z), (x, y, pre_z)], QG)
    log("-> pre-grasp"); go(r, pre, 6.0)

    # descend in 3 steps
    zs = np.linspace(pre_z, grasp_z, 5)[1:]
    down = cart_path(r, pre[-1], [(x, y, z) for z in zs], QG)
    log("-> descend"); tcp = go(r, down, 2.5)
    for i in range(3):
        off = np.linalg.norm(tcp[:2] - np.array([x, y]))
        if off <= 0.006 and abs(tcp[2] - grasp_z) <= 0.01:
            break
        log(f"  correcting: off={off:.4f} dz={tcp[2]-grasp_z:.4f}")
        fix = cart_path(r, r.joints(), [(x, y, grasp_z)], QG)
        tcp = go(r, fix, 2.0)
    if np.linalg.norm(tcp[:2] - np.array([x, y])) > 0.01:
        log("  ABORT: xy still off", np.round(tcp[:2] - [x, y], 3)); sys.exit(3)

    log("close gripper"); res = r.gripper(0.0); log(" ", res)
    f1, f2 = res[2]
    gap = abs(f1) + abs(f2)
    log(f"  finger gap={gap:.4f}")
    if gap < 0.01:
        log("  GRASP FAILED (closed on air)"); sys.exit(2)

    up = cart_path(r, r.joints(), [(x, y, grasp_z + 0.08), (x, y, lift_z)], QG)
    log("-> lift"); go(r, up, 3.0)
    log(f"  fingers after lift={r.fingers()}")

    over = cart_path(r, up[-1], [(BASKET[0], BASKET[1], lift_z)])
    log("-> over basket"); go(r, over, 6.0)
    log(f"  fingers over basket={r.fingers()}")
    lower = cart_path(r, over[-1], [(BASKET[0], BASKET[1], 0.70)])
    log("-> lower into basket"); go(r, lower, 2.0)

    log("open gripper"); log(" ", r.gripper(0.04))
    back = cart_path(r, lower[-1], [(BASKET[0], BASKET[1], lift_z + 0.04)])
    log("-> retreat"); go(r, back, 2.0)
    log(f"[{name}] DONE fingers={r.fingers()}")


if __name__ == "__main__":
    main()
