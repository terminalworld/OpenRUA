from rlib import *
from scene import *
r = Robot("tp2")
sc = Scene(r.node)
th = np.deg2rad(25)
Rplace = hand_R([1, 0, 0], [0, -np.sin(th), -np.cos(th)])
yp = -0.155
seed = r.arm_q()
for xh in [-0.16, -0.155, -0.15, -0.145, -0.14, -0.13]:
    for z in [0.975, 0.985, 0.995]:
        try:
            q = r.ik([xh, yp, z], Rplace, seed=seed); seed = q
            v, c = sc.check(ARM, q, 0.04)
            print(f"hand x={xh:.3f} z={z:.3f} valid={v} {c}")
        except Exception as e:
            print(f"hand x={xh:.3f} z={z:.3f} IK error {e}")
