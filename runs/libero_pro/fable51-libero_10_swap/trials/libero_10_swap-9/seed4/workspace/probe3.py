import numpy as np, sys
from rob import Robot, quat_from_axes
r = Robot("probe3")
q0 = r.q()
tip = np.array([-0.156, -0.494, 0.948])
for th in [25, 30, 35, 40]:
    t = np.radians(th)
    for yaw in [0, -10, -20]:   # yaw: rotate pointing dir about z (negative -> toward -x)
        yw = np.radians(yaw)
        d = np.array([-np.sin(yw), np.cos(yw), 0])   # horizontal pointing dir
        z = d*np.sin(t) + np.array([0,0,-np.cos(t)])
        x = d*np.cos(t) + np.array([0,0,np.sin(t)])
        quat = quat_from_axes(z, x)
        seed = q0
        out = []
        for name, p in [("pre", tip - 0.06*z), ("grasp", tip), ("lift", tip + [0,0,0.17])]:
            q = r.solve_ik(p, quat, seed=seed, tries=6)
            if q is None:
                out.append(f"{name}:--"); continue
            seed = q
            l7 = r.fk_pose(q, "panda_link7")[0]
            out.append(f"{name}:OK l7={np.round(l7,3)} q={np.round(q,2)}")
        print(f"th={th} yaw={yaw} | " + " | ".join(out)); sys.stdout.flush()
