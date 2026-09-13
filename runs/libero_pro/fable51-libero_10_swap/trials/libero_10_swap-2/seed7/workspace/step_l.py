import numpy as np
from lib import Robot
r = Robot("stepl")
quat = (0.8315, 0.5556, 0, 0)
f0 = r.wrench()[0]
for z in [1.157, 1.152, 1.147, 1.142]:
    q = r.ik_world([-0.058, 0.20, z], quat); code, err = r.move_q(q, 1.0)
    p, _, _ = r.fk_world(); f = r.wrench()[0]
    print(f"  target z={z} at {np.round(p,4)} err={err:.4f} wrench {np.round(f,2)} dz={f[2]-f0[2]:.2f}")
    if f[2] - f0[2] > 1.5 or err > 0.005:
        print("contact -> stop"); break
r.gripper(0.04)
print("gap after open:", round(r.finger_gap(),4))
p, _, _ = r.fk_world()
q = r.ik_world(p + [0,0,0.12], quat); r.move_q(q, 2.0)
print("retreated to", np.round(r.fk_world()[0],4))
