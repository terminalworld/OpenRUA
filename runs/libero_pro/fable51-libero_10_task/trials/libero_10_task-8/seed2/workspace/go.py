#!/usr/bin/env python3
"""Stage runner: python3 go.py <stage> [args]. Poses in world frame."""
import sys, json
import numpy as np
from rob import *

R_DOWN_X = frame_from_axes([1, 0, 0], [0, 0, -1])   # fingers close along world x, hand points down
Q_DOWN_X = R_to_quat(R_DOWN_X)


def goto(r, p_flange, quat, secs, seed=None, via=None):
    """IK then one trajectory (optionally through a via joint config)."""
    sol = r.ik_world(p_flange, quat, seed=r.arm_q() if seed is None else seed, timeout=3.0)
    if sol is None:
        for s in ([0, -0.785, 0, -2.356, 0, 1.571, 0.785], [0.3, 0.5, 0, -1.8, 0, 2.3, 0.8]):
            sol = r.ik_world(p_flange, quat, seed=s, timeout=3.0)
            if sol is not None:
                break
    if sol is None:
        raise SystemExit(f"IK failed for {p_flange}")
    print("  target q", np.round(sol, 3), flush=True)
    if via is not None:
        code, err = r.move_joints([via, sol], [secs * 0.5, secs])
    else:
        code, err = r.move_joints([sol], [secs])
    for _ in range(4):                      # controller lag: resend short goals until converged
        if err < 0.01:
            break
        code, err = r.move_joints([sol], [2.0])
    if err >= 0.01:
        raise SystemExit(f"did not converge: max|dq|={err:.4f}")
    p, q = r.fk_world()
    print("  now flange", np.round(p, 4), "quat", np.round(q, 3), flush=True)
    return sol


def tcp_to_flange(p_tcp, quat):
    R = quat_to_R(quat)
    return np.asarray(p_tcp) - TCP * R[:, 2]


if __name__ == "__main__":
    r = Robot("go")
    st = sys.argv[1]
    a = [float(x) for x in sys.argv[2:]]
    if st == "tcp":            # go.py tcp x y z [secs]  (fingers along x, pointing down)
        secs = a[3] if len(a) > 3 else 4.0
        goto(r, tcp_to_flange(a[:3], Q_DOWN_X), Q_DOWN_X, secs)
    elif st == "tcpq":         # go.py tcpq x y z qx qy qz qw [secs]
        q = np.array(a[3:7]); secs = a[7] if len(a) > 7 else 4.0
        goto(r, tcp_to_flange(a[:3], q), q, secs)
    elif st == "grip":
        r.gripper(a[0])
    elif st == "state":
        p, q = r.fk_world()
        print("flange", np.round(p, 4), "quat", np.round(q, 4), "tcp", np.round(p + TCP * quat_to_R(q)[:, 2], 4))
        print("q", np.round(r.arm_q(), 4), "gap", round(r.finger_gap(), 4))
