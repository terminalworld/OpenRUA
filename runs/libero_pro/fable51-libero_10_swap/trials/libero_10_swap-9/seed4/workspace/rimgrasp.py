import numpy as np, sys, time
from rob import Robot, quat_from_axes
from scipy.spatial.transform import Rotation as Rot

def rim_quat(th, yaw, rho):
    t, yw, rh = np.radians(th), np.radians(yaw), np.radians(rho)
    d = np.array([-np.sin(yw), np.cos(yw), 0])
    z = d*np.sin(t) + np.array([0, 0, -np.cos(t)])
    x0 = d*np.cos(t) + np.array([0, 0, np.sin(t)])
    y0 = np.cross(z, x0)
    y = np.cos(rh)*y0 + np.sin(rh)*x0      # closing axis rotated about z by rho
    x = np.cross(y, z)
    return quat_from_axes(z, x), z, y

if __name__ == "__main__":
    mode = sys.argv[1]
    th, yaw, rho = 40, -20, 15
    quat, z, c = rim_quat(th, yaw, rho)
    print("z", np.round(z,3), "closing", np.round(c,3))
    tip = np.array([float(v) for v in sys.argv[2:5]])
    r = Robot("rim")
    q0 = r.q()
    targets = {"pre": tip - 0.06*z, "grasp": tip, "lift": tip + [0,0,0.02], "lift2": tip + [0,0,0.17]}
    qs = {}
    seed = q0
    for k in ["pre", "grasp", "lift", "lift2"]:
        q = r.solve_ik(targets[k], quat, seed=seed, tries=8)
        print(k, np.round(targets[k],3), "OK" if q is not None else "FAIL")
        if q is None: sys.exit(1)
        qs[k] = q; seed = q
    if mode == "check": sys.exit(0)
    def go(k, sec):
        for _ in range(3):
            code, err = r.move_joints(qs[k], seconds=sec)
            if err < 0.01: break
        p, _ = r.tcp_pose()
        print(k, "code", code, "err", round(err,4), "tcp", np.round(p,4).tolist(), "fingers", np.round(r.fingers(),4))
        return p
    go("pre", 4)
    p = go("grasp", 4)
    # correct offset once
    off = p - tip
    if np.linalg.norm(off) > 0.003:
        q = r.solve_ik(tip - off, quat, seed=r.q(), tries=8)
        if q is not None:
            qs["grasp2"] = q; go("grasp2", 2)
    print("close", r.gripper(0.0))
    go("lift", 3)
    f = r.fingers()
    if f[0] < 0.0015:
        print("SLIPPED"); sys.exit(2)
    go("lift2", 5)
    print("final fingers", r.fingers())
