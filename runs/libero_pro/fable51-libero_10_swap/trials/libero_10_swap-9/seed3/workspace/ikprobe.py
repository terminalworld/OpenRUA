import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot()
print("current hand fk:", np.round(r.fk()[0],3))
seed=r.arm_q()
cands = {
 "above mug, vertical, fingers x": ((-0.085,0.026,1.16), quat_from_axes((0,0,-1),(1,0,0))),
 "above mug, vertical, fingers y": ((-0.012,0.076,1.16), quat_from_axes((0,0,-1),(0,1,0))),
 "front of door vertical (-0.085,-0.39,1.16)": ((-0.085,-0.39,1.16), quat_from_axes((0,0,-1),(1,0,0))),
 "front of door vertical (-0.135,-0.42,1.16)": ((-0.135,-0.42,1.16), quat_from_axes((0,0,-1),(1,0,0))),
 "horiz +y at (-0.135,-0.44,0.98)": ((-0.135,-0.44,0.98), quat_from_axes((0,1,0),(1,0,0))),
 "horiz +y at (-0.135,-0.40,1.0)": ((-0.135,-0.40,1.0), quat_from_axes((0,1,0),(1,0,0))),
 "horiz +y at (-0.135,-0.36,1.0)": ((-0.135,-0.36,1.0), quat_from_axes((0,1,0),(1,0,0))),
 "horiz +y at (-0.135,-0.50,1.0)": ((-0.135,-0.50,1.0), quat_from_axes((0,1,0),(1,0,0))),
 "horiz +y at (-0.135,-0.55,1.0)": ((-0.135,-0.55,1.0), quat_from_axes((0,1,0),(1,0,0))),
 "horiz (+x+y) at (-0.135,-0.50,1.0)": ((-0.135,-0.50,1.0), quat_from_axes((0.5,0.866,0),(0.866,-0.5,0))),
 "pitched 45 +y at (-0.135,-0.50,1.05)": ((-0.135,-0.50,1.05), quat_from_axes((0,0.707,-0.707),(1,0,0))),
 "horiz +y fingers z at (-0.135,-0.44,1.0)": ((-0.135,-0.44,1.0), quat_from_axes((0,1,0),(0,0,1))),
 "door push vertical (-0.30,-0.50,1.0)": ((-0.30,-0.50,1.0), quat_from_axes((0,0,-1),(1,0,0))),
 "door push vertical (-0.15,-0.55,1.0)": ((-0.15,-0.55,1.0), quat_from_axes((0,0,-1),(1,0,0))),
 "door push vertical (-0.02,-0.45,1.0)": ((-0.02,-0.45,1.0), quat_from_axes((0,0,-1),(1,0,0))),
}
for k,(p,q) in cands.items():
    sol=r.ik(p,q,seed=seed)
    if sol is None:
        # retry with a different seed
        sol=r.ik(p,q,seed=[0,-0.5,0,-2.0,0,1.6,0.8])
    print(f"{k:50s} ->", "FAIL" if sol is None else np.round(sol,2))
rclpy.shutdown()
