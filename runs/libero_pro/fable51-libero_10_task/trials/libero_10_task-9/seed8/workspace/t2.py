from lib import *
import random
r = Robot("t2")
q0 = r.arm_q()
def Rpitch(pitch_deg, yaw_axis='+y'):
    # approach axis: +y rotated downward by pitch about x
    th = np.radians(pitch_deg)
    z_h = np.array([0, np.cos(th), -np.sin(th)])
    y_h = np.array([1.,0,0])
    x_h = np.cross(y_h, z_h)
    return R_from_axes(x_h, y_h, z_h)
seeds = [q0] + [[random.uniform(*l) for l in FJT["limits_rad"]] for _ in range(6)]
for pitch in [0, 15, 30, 45]:
    R = Rpitch(pitch)
    for pt in [(-0.10,-0.36,0.96),(-0.10,-0.36,1.0),(-0.10,-0.40,1.0),(-0.10,-0.34,0.97)]:
        ok=None
        for s in seeds:
            sol = r.ik(np.array(pt), R, seed=s, timeout=1.0)
            if sol: ok=sol; break
        print(f"pitch={pitch} pt={pt} -> {None if ok is None else np.round(ok,3)}", flush=True)
