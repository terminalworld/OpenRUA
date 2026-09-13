"""Checked, paced motion on top of kin.py (local IK) and ctl.Ctl."""
import numpy as np
import kin
from kin import Rot
from ctl import Ctl

VMAX = 0.15  # rad/s per joint: the controller silently caps near ~0.19


def R_from_axes(z, y):
    z = np.asarray(z, float); z /= np.linalg.norm(z)
    y = np.asarray(y, float); y = y - (y @ z) * z; y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


def cart_path(q0, pos1, R1, use_tcp=True, step_m=0.03, step_deg=10, null_bias=0.05):
    """Straight-line (pos slerp) TCP/hand path from fk(q0) to (pos1,R1) as joint waypoints."""
    p0, R0 = (kin.tcp if use_tcp else kin.hand)(q0)
    pos1 = np.asarray(pos1, float)
    d = np.linalg.norm(pos1 - p0)
    ang = np.degrees((Rot.from_matrix(R0).inv() * Rot.from_matrix(R1)).magnitude())
    n = max(1, int(np.ceil(max(d / step_m, ang / step_deg))))
    rots = Rot.from_matrix(np.stack([R0, R1]))
    qs, q = [], np.array(q0, float)
    for i in range(1, n + 1):
        t = i / n
        p = p0 + t * (pos1 - p0)
        # slerp between two rotations
        R = (Rot.from_matrix(R0) * Rot.from_rotvec(
            t * (Rot.from_matrix(R0).inv() * Rot.from_matrix(R1)).as_rotvec())).as_matrix()
        q = kin.ik(p, R, q, use_tcp=use_tcp, null_bias=null_bias)
        qs.append(q)
    return qs


def check(qs, q0, ignore=(), skip_pts=(), hand_pts=kin.HAND_PTS):
    hits = []
    prev = np.array(q0)
    for q in qs:
        hits += kin.collisions(prev, q, n=8, ignore=ignore, skip_pts=skip_pts, hand_pts=hand_pts)
        prev = q
    return hits


def execute(c: Ctl, qs, ignore=(), skip_pts=(), hand_pts=kin.HAND_PTS, vmax=VMAX, label=""):
    q0 = np.array(c.arm_q())
    hits = check(qs, q0, ignore, skip_pts, hand_pts)
    if hits:
        seen = {}
        for h in hits:
            seen.setdefault((h[1], h[2]), h)
        raise RuntimeError(f"{label}: path hits {list(seen.values())[:6]}")
    # pacing by max joint delta per segment
    times, t, prev = [], 0.0, q0
    for q in qs:
        t += max(0.5, np.abs(np.array(q) - prev).max() / vmax)
        times.append(t); prev = np.array(q)
    code, err = c.movej([list(q) for q in qs], times)
    if code != 0 or err > 0.02:
        # one paced resend of the final target
        q_now = np.array(c.arm_q())
        code, err = c.movej(list(qs[-1]), max(1.0, np.abs(np.array(qs[-1]) - q_now).max() / vmax))
    q_now = np.array(c.arm_q())
    p, R = kin.tcp(q_now)
    print(f"  [{label}] done code={code} jerr={err:.4f} tcp={p.round(4)} Z={R[:,2].round(2)} Y={R[:,1].round(2)}")
    if err > 0.02:
        raise RuntimeError(f"{label}: did not converge, joint err {err:.3f}")
    return q_now


def goto(c, pos, R, use_tcp=True, label="", **kw):
    q0 = np.array(c.arm_q())
    qs = cart_path(q0, pos, R, use_tcp=use_tcp)
    return execute(c, qs, label=label, **kw)


def gotoj(c, q, label="", **kw):
    return execute(c, [np.array(q)], label=label, **kw)
