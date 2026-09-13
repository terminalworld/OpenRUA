import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
r = Robot("iktest2")
def Q(z, y):
    z = np.array(z, float); y = np.array(y, float); x = np.cross(y, z)
    return Rot.from_matrix(np.stack([x, y, z], 1)).as_quat()
cases = {
 "+y horiz, pinch up, (-0.19,-0.33,0.98)": ((-0.19,-0.33,0.98), Q([0,1,0],[0,0,1])),
 "+y horiz, pinch dn, (-0.19,-0.33,0.98)": ((-0.19,-0.33,0.98), Q([0,1,0],[0,0,-1])),
 "-y horiz, pinch up, (-0.19,-0.05,0.98)": ((-0.19,-0.05,0.98), Q([0,-1,0],[0,0,1])),
 "-y horiz, pinch dn, (-0.19,-0.05,0.98)": ((-0.19,-0.05,0.98), Q([0,-1,0],[0,0,-1])),
 "+x horiz, pinch up, (-0.15,-0.19,0.98)": ((-0.15,-0.19,0.98), Q([1,0,0],[0,0,1])),
 "+x horiz, pinch dn, (-0.15,-0.19,0.98)": ((-0.15,-0.19,0.98), Q([1,0,0],[0,0,-1])),
 "+x horiz, pinch up, (-0.05,-0.19,0.98)": ((-0.05,-0.19,0.98), Q([1,0,0],[0,0,1])),
}
seeds = [r.arm_q(), [0,-0.785,0,-2.356,0,1.571,0.785], [-0.5,0.3,0.2,-2.0,0,2.3,0.5], [0.5,0.5,-0.5,-1.8,0.3,2.4,1.0]]
for name,(p,q) in cases.items():
    sols=[]
    for s in seeds:
        sol = r.ik(p, q, seed=s, timeout=0.5)
        if sol: sols.append(np.round(sol,2))
    print(name, "->", len(sols), "sols", sols[:2])
