import numpy as np, sys
from rob import Robot, quat_from_axes
r = Robot("doorplan")
H = np.array([-0.18, 0.26]); S = 0.20; Z = 1.00; DELTA = 0.015; PITCH = np.deg2rad(45)
def door_pose(alpha_deg, delta=DELTA):
    a = np.deg2rad(alpha_deg)
    d = np.array([np.cos(a), np.sin(a), 0.0]); n = np.array([np.sin(a), -np.cos(a), 0.0])
    p = np.array([*(H + S * d[:2]), Z])
    tcp = p + delta * n
    z_h = np.cos(PITCH) * (-n) - np.sin(PITCH) * np.array([0, 0, 1.0])
    y_h = d; x_h = np.cross(y_h, z_h)
    return tcp, quat_from_axes(x_h, y_h, z_h)
if __name__ == "__main__":
    seed = r.arm_q(); prev = None
    alphas = [-125] + list(range(-125, 1, 5))
    deltas = [0.06] + [DELTA] * (len(alphas) - 1)
    for a, dl in zip(alphas, deltas):
        tcp, q = door_pose(a, dl)
        sol = r.ik(tcp, q, seed=seed, timeout=3.0)
        if sol is None:
            print(f"alpha {a:5d} delta {dl:.3f} tcp {tcp.round(3)} IK FAIL"); continue
        jump = 0 if prev is None else np.max(np.abs(np.array(sol) - np.array(prev)))
        print(f"alpha {a:5d} tcp {tcp.round(3)} q {np.round(sol,2)} maxjump {jump:.2f}")
        seed = sol; prev = sol
