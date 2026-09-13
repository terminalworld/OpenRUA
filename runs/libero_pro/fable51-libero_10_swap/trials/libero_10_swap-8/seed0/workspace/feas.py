"""Brute-force feasible top-down-ish pinch orientations for a lying pot.
usage: feas.py wx wy wz ax ay [up]  (waist point, horizontal axis dir bottom->lid)
Prints feasible (gamma, sgn, pitch) with the lowest finger point; y = sgn*(cos g*h + sin g*Z)."""
import math, sys, numpy as np
import task, kin
from kin import ik
from rob import R_to_quat
from task import HOME, TCP_OFF

p = np.array(list(map(float, sys.argv[1:4]))) if len(sys.argv) > 5 else None
a = (np.array([float(sys.argv[4]), float(sys.argv[5]), 0.0]) if len(sys.argv) > 5 else np.array([1.0, 0, 0])); a /= np.linalg.norm(a)
up = float(sys.argv[6]) if len(sys.argv) > 6 else 0.008
Z = np.array([0, 0, 1.0]); h = np.cross(a, Z)


def hand_R(gam, sgn, pitch):
    y = sgn * (math.cos(gam) * h + math.sin(gam) * Z)
    zd = -Z - (-Z @ y) * y; zd /= np.linalg.norm(zd)
    z = math.cos(pitch) * zd + math.sin(pitch) * a
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


if __name__ == "__main__":
    n = 0
    for gam in range(-60, 61, 15):
        for sgn in (1, -1):
            for bdeg in range(-60, 61, 15):
                R = hand_R(math.radians(gam), sgn, math.radians(bdeg))
                if R[2, 2] > -0.4:
                    continue
                z = R[:, 2]; tcp = p - up * z; pos = tcp - TCP_OFF * z
                q = ik(pos, R_to_quat(R), HOME, w_post=0.03, tries=4)
                if q is None:
                    continue
                qa = ik(pos - 0.10 * z, R_to_quat(R), q, w_post=0.03, tries=4)
                if qa is None:
                    continue
                pts = [tcp + s * R[:, 1] + t * z + u * R[:, 0] for s in (-0.039, 0.039) for t in (0.0, 0.01) for u in (-0.009, 0.009)]
                low = min(pts, key=lambda v: v[2])
                n += 1
                print(f"gamma={gam:+d} sgn={sgn:+d} pitch={bdeg:+d}: lowest finger {np.round(low,3)} hand z={np.round(z,2)} q={np.round(q,2)}", flush=True)
    print("feasible:", n)
