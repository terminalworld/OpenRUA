import numpy as np, sys
from rob import Robot, quat_from_axes
H = np.array([-0.18, 0.26]); S = 0.12; ZT = 1.085; W = 0.017
def door_pose(alpha_deg, w=W, z=ZT, s=S):
    a = np.deg2rad(alpha_deg)
    d = np.array([np.cos(a), np.sin(a), 0.0]); n = np.array([np.sin(a), -np.cos(a), 0.0])
    tcp = np.array([*(H + s * d[:2]), z]) + w * n
    z_h = np.array([0, 0, -1.0]); y_h = d; x_h = np.cross(y_h, z_h)
    return tcp, quat_from_axes(x_h, y_h, z_h)
def run_arc(r, a0, a1, step=5, per=1.0):
    alphas = list(np.arange(a0, a1 + 1e-6, step if a1 > a0 else -step))
    qs = []; seed = r.arm_q()
    for a in alphas:
        tcp, q = door_pose(a)
        sol = r.ik(tcp, q, seed=seed, timeout=3.0)
        if sol is None: raise SystemExit(f"IK fail at alpha {a}")
        if qs and np.max(np.abs(np.array(sol) - np.array(qs[-1]))) > 0.6:
            raise SystemExit(f"joint jump at alpha {a}: {np.round(np.array(sol)-np.array(qs[-1]),2)}")
        qs.append(sol); seed = sol
    r.move_traj(qs, [per * (i + 1) for i in range(len(qs))])
    p, _ = r.tcp(); print("arc end TCP", p.round(4), "target", door_pose(a1)[0].round(4))
if __name__ == "__main__":
    r = Robot("door")
    mode = sys.argv[1]
    if mode == "approach":
        A0 = float(sys.argv[2])
        r.gripper(0.0)
        for (w, z) in [(0.05, 1.30), (0.05, ZT), (W, ZT)]:
            tcp, q = door_pose(A0, w=w, z=z)
            sol = r.ik(tcp, q, seed=r.arm_q(), timeout=3.0); assert sol is not None
            r.move_q_corrected(sol, seconds=4.0, iters=2)
            print("TCP", r.tcp()[0].round(4), "q", np.round(r.arm_q(), 2))
    elif mode == "arc":
        run_arc(r, float(sys.argv[2]), float(sys.argv[3]))
    elif mode == "up":
        p, q = r.tcp(); tcp = p.copy(); tcp[2] = 1.30
        sol = r.ik(tcp, q, seed=r.arm_q()); r.move_q_corrected(sol, seconds=3.0, iters=1)
        print("TCP", r.tcp()[0].round(4))
