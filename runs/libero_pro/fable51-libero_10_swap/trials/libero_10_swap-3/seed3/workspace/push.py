"""Close the drawer: horizontal push on the bottom-drawer handle.
Usage: python3 -u push.py check|run [stage...]"""
import sys
import numpy as np
from robot import *
from moveit_msgs.srv import GetPositionFK

PSI, PHI = np.radians(25), np.radians(15)   # yaw toward +x of the approach, pitch down
PUSH_X = -0.12
PUSH_Z = 0.947                               # handle bar centre height
Y_PRE = 0.0                                  # fingertips 3 cm in front of the handle face (-0.03)
Y_END = -0.183                               # handle face at -0.19 when the drawer is closed (face now -0.028)


def push_quat(psi=PSI, phi=PHI):
    d = np.array([np.sin(psi) * np.cos(phi), -np.cos(psi) * np.cos(phi), -np.sin(phi)])  # hand z
    xh = np.array([np.sin(psi) * np.sin(phi), -np.cos(psi) * np.sin(phi), np.cos(phi)])  # hand x (up-ish)
    yh = np.cross(d, xh)
    R = np.column_stack([xh, yh, d])
    t = np.trace(R)
    w = np.sqrt(1 + t) / 2
    return (float((R[2, 1] - R[1, 2]) / (4 * w)), float((R[0, 2] - R[2, 0]) / (4 * w)),
            float((R[1, 0] - R[0, 1]) / (4 * w)), float(w))


QP = push_quat()
SEED_B = [0.31, 0.92, 0.26, -1.74, -1.13, 1.43, -2.71]

# ---- obstacles as axis-aligned boxes (world) ----
OBST = {
    "table": (-1.0, 1.0, -1.0, 1.0, 0.0, 0.90),
    "stand_lo": (-0.134, 0.136, 0.185, 0.215, 0.90, 1.03),   # board edge is low near y=0.19
    "stand": (-0.134, 0.136, 0.215, 0.36, 0.90, 1.25),
    "bottle": (-0.025, 0.05, -0.03, 0.05, 0.90, 1.065),
    "cabinet": (-0.30, 0.02, -0.60, -0.19, 0.90, 1.135),   # body + protruding handles
    "drawer": (-0.23, 0.01, -0.225, -0.03, 0.90, 0.985),   # open drawer + its handle
}
LINK_R = {"panda_link5": 0.06, "panda_link6": 0.06, "panda_link7": 0.055}


def link_pts(r, q):
    """Sample points of the arm's distal links + hand + fingers in world."""
    req = GetPositionFK.Request()
    req.header.frame_id = ""
    req.fk_link_names = list(LINK_R) + ["panda_hand"]
    req.robot_state.joint_state.name = list(ARM)
    req.robot_state.joint_state.position = [float(v) for v in q]
    fut = r.fk_cli.call_async(req)
    rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    pts = []
    for name, ps in zip(req.fk_link_names, fut.result().pose_stamped):
        p = np.array([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z])
        if name in LINK_R:
            rad = LINK_R[name]
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        v = np.array([dx, dy, dz], float)
                        if np.linalg.norm(v) > 0:
                            v = v / np.linalg.norm(v) * rad
                        pts.append((name, p + v))
        else:
            o = ps.pose.orientation
            R = quat_to_R((o.x, o.y, o.z, o.w))
            for hx in (-0.035, 0.035):
                for hy in (-0.10, 0.10):
                    for hz in (0.0, 0.066):
                        pts.append(("palm", p + R @ [hx, hy, hz]))
            for hy in (-0.045, 0.045):          # fingers (open-ish envelope)
                for hz in (0.058, 0.115):
                    pts.append(("finger", p + R @ [0.0, hy, hz]))
            pts.append(("camera", p + R @ [0.06, 0.0, 0.0]))
    return pts


def check_path(r, q0, q1, steps=12, ignore=()):
    """Linear joint interpolation; report obstacle hits."""
    hits = []
    for i in range(steps + 1):
        q = np.array(q0) + (np.array(q1) - np.array(q0)) * i / steps
        for name, p in link_pts(r, q):
            for ob, (x0, x1, y0, y1, z0, z1) in OBST.items():
                if ob in ignore:
                    continue
                if x0 <= p[0] <= x1 and y0 <= p[1] <= y1 and z0 <= p[2] <= z1:
                    hits.append((i, name, ob, p.round(3)))
    return hits


def plan(r):
    """IK for all waypoints; returns list of (label, q, secs)."""
    q_now = r.arm_q()
    wps = []
    q = r.ik((PUSH_X, Y_PRE, 1.12), Q_DOWN_FX, seed=q_now)
    assert q is not None, "IK wp1"
    wps.append(("over_prepush_down", q, 4.0))
    q = r.ik((PUSH_X, Y_PRE, 1.00), QP, seed=SEED_B)
    assert q is not None, "IK wp2"
    wps.append(("reorient_horizontal", q, 4.0))
    seed = q
    q = r.ik((PUSH_X, Y_PRE, PUSH_Z), QP, seed=seed)
    assert q is not None, "IK wp3"
    wps.append(("descend_prepush", q, 3.0))
    seed = q
    line = []
    for y in np.linspace(Y_PRE, Y_END, 7)[1:]:
        q = r.ik((PUSH_X, y, PUSH_Z), QP, seed=seed)
        assert q is not None, f"IK push y={y}"
        seed = q
        line.append(q)
    wps.append(("push", line, 6.0))
    return wps


def do_check(r, wps):
    q_prev = r.arm_q()
    ok = True
    for label, q, _ in wps:
        qs = q if isinstance(q, list) and isinstance(q[0], (list, np.ndarray)) else [q]
        for k, qq in enumerate(qs):
            ign = ("drawer",) if label == "push" else ()
            hits = check_path(r, q_prev, qq, ignore=ign)
            margin = min(min(v - lo, hi - v) for v, (lo, hi) in zip(qq, FJT["limits_rad"]))
            tp, _ = r.tcp(qq)
            print(f"{label}[{k}]: tcp={tp.round(3)} q={np.round(qq, 2)} jmargin={margin:.2f} hits={len(hits)}")
            for h in hits[:8]:
                print("    ", h)
            ok &= not hits
            q_prev = qq
    return ok


if __name__ == "__main__":
    r = Robot()
    print("QP", np.round(QP, 4))
    print(quat_to_R(QP).round(3))
    wps = plan(r)
    mode = sys.argv[1]
    if mode == "check":
        print("ALL CLEAR" if do_check(r, wps) else "COLLISION RISK")
    elif mode == "run":
        wanted = sys.argv[2:] or [w[0] for w in wps]
        for label, q, secs in wps:
            if label not in wanted:
                continue
            print("==", label)
            if label == "push":
                n = len(q)
                r.move_joints(q, [secs * (i + 1) / n for i in range(n)])
            else:
                r.move_joints([q], [secs])
            tp, _ = r.tcp()
            print(f"   TCP now {tp.round(4)} fingers {np.round(r.fingers(), 4)}")
