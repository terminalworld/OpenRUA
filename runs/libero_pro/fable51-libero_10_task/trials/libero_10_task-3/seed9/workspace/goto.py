"""Robust TCP goto: IK (verified by FK), move, measure, resend until within tol."""
import numpy as np
from rob import *

LIM = np.array(M["actuators"][0]["limits_rad"])


def margin(q):
    return float(min((q - LIM[:, 0]).min(), (LIM[:, 1] - q).min()))


def ik_checked(r, tcp, R, seed, max_dist=0.8, tries=6):
    best = None
    for i in range(tries):
        s = seed if i == 0 else seed + np.random.uniform(-0.15, 0.15, 7)
        q = r.ik_tcp(tcp, R, seed=s)
        if q is None:
            continue
        t, Rq = r.tcp(q)
        if np.abs(t - tcp).max() > 0.003 or np.abs(Rq - R).max() > 0.05:
            continue
        d = np.abs(q - seed).max()
        if d < max_dist and margin(q) > 0.1:
            if best is None or d < best[0]:
                best = (d, q)
    return None if best is None else best[1]


def goto(r, tcp, R, seconds=4.0, tol=0.008, max_dist=0.8, resend=3, via_n=0):
    tcp = np.asarray(tcp, float)
    q0 = r.arm_q()
    if via_n > 0:
        t0, R0 = r.tcp()
        path = cart_path(r, t0, R0, tcp, R, via_n + 1, q0, max_step=0.5)
        if path is None:
            print("cart_path failed, falling back to direct IK", flush=True)
            via_n = 0
        else:
            q = path[-1]
            via = path[:-1]
    if via_n == 0:
        q = ik_checked(r, tcp, R, q0, max_dist=max_dist)
        via = None
        if q is None:
            print(f"IK failed for {np.round(tcp,3)}", flush=True)
            return None
    print(f"goto {np.round(tcp,3)}: dq={np.round(np.abs(q-q0).max(),3)} margin={margin(q):.2f}", flush=True)
    r.move_q(q, seconds, via=via)
    for i in range(resend):
        t, _ = r.tcp()
        err = np.abs(t - tcp).max()
        print(f"  tcp {np.round(t,4)} err={err:.4f}", flush=True)
        if err < tol:
            break
        r.move_q(q, max(2.0, seconds / 2))
    return q
