"""Multi-seed feasibility scan. usage: feas2.py wx wy wz ax ay [up] [above]"""
import math, sys, numpy as np
from kin import ik
from rob import R_to_quat
from task import HOME, TCP_OFF
from feas import hand_R as _hr
import feas

def hand_R(a, gam, sgn, pitch):
    Z = np.array([0, 0, 1.0]); h = np.cross(a, Z)
    y = sgn * (math.cos(gam) * h + math.sin(gam) * Z)
    zd = -Z - (-Z @ y) * y; zd /= np.linalg.norm(zd)
    z = math.cos(pitch) * zd + math.sin(pitch) * a
    x = np.cross(y, z)
    return np.column_stack([x, y, z])

SEEDS = [HOME, np.array([0.2,1.2,-0.3,-0.8,-0.5,2.8,-0.5]), np.array([0.3,1.5,-0.7,-0.5,-0.3,3.0,-0.3]),
         np.array([-0.3,1.4,0.6,-0.7,-0.2,2.7,-1.5]), np.array([0.0,1.0,0.0,-1.2,0.0,2.2,0.8]),
         np.array([0.32,1.68,-0.69,-0.5,-0.27,3.05,-0.29]), np.array([-0.06,1.38,0.04,-0.75,-0.64,2.77,-0.69]),
         np.array([0.46,1.65,-1.11,-0.59,-1.33,3.29,0.16])]

def solve(pos, R, tries=2):
    for s in SEEDS:
        q = ik(pos, R_to_quat(R), s, w_post=0.03, tries=tries)
        if q is not None:
            return q
    return None

if __name__ == "__main__":
    p = np.array(list(map(float, sys.argv[1:4])))
    a = np.array([float(sys.argv[4]), float(sys.argv[5]), 0.0]); a /= np.linalg.norm(a)
    up = float(sys.argv[6]) if len(sys.argv) > 6 else 0.008
    above = float(sys.argv[7]) if len(sys.argv) > 7 else 0.08
    n = 0
    for gam in range(-60, 61, 15):
        for sgn in (1, -1):
            for bdeg in range(-75, 76, 15):
                R = hand_R(a, math.radians(gam), sgn, math.radians(bdeg))
                if R[2, 2] > -0.3:
                    continue
                z = R[:, 2]; tcp = p - up * z; pos = tcp - TCP_OFF * z
                q = solve(pos, R)
                if q is None:
                    continue
                qa = solve(pos - above * z, R)
                pts = [tcp + s * R[:, 1] + t * z + u * R[:, 0] for s in (-0.051, 0.051) for t in (0.0, 0.01) for u in (-0.009, 0.009)]
                low = min(pts, key=lambda v: v[2])
                n += 1
                print(f"gamma={gam:+d} sgn={sgn:+d} pitch={bdeg:+d}: above {'OK' if qa is not None else '--'} lowest corner {np.round(low,3)} hand z={np.round(z,2)} q={np.round(q,2)}", flush=True)
    print("feasible:", n)
