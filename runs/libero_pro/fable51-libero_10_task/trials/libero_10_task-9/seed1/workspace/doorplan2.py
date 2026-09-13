import numpy as np, sys
from rob import Robot, quat_from_axes
H = np.array([-0.18, 0.26]); Z = 1.00; DELTA = 0.015; PITCH = np.deg2rad(45)
def door_pose(alpha_deg, s, delta=DELTA, z=Z):
    a = np.deg2rad(alpha_deg)
    d = np.array([np.cos(a), np.sin(a), 0.0]); n = np.array([np.sin(a), -np.cos(a), 0.0])
    tcp = np.array([*(H + s * d[:2]), z]) + delta * n
    z_h = np.cos(PITCH) * (-n) - np.sin(PITCH) * np.array([0, 0, 1.0])
    y_h = d; x_h = np.cross(y_h, z_h)
    return tcp, quat_from_axes(x_h, y_h, z_h)
if __name__ == "__main__":
    r = Robot("doorplan2")
    S = float(sys.argv[1])
    for seed0 in ([0.5, 0.2, -0.3, -2.2, 0.0, 2.4, 0.8], [0.3, -0.6, 0.2, -2.8, 0.1, 2.3, 0.9], r.arm_q()):
        print("seed", np.round(seed0, 2)); seed = seed0; prev = None
        for a in [-125, -110, -90, -70, -50, -30, -15, 0]:
            tcp, q = door_pose(a, S)
            sol = r.ik(tcp, q, seed=seed, timeout=3.0)
            if sol is None: print(f"  alpha {a:5d} tcp {tcp.round(3)} FAIL"); continue
            jump = 0 if prev is None else np.max(np.abs(np.array(sol) - np.array(prev)))
            print(f"  alpha {a:5d} tcp {tcp.round(3)} q {np.round(sol,2)} jump {jump:.2f}")
            seed = sol; prev = sol
