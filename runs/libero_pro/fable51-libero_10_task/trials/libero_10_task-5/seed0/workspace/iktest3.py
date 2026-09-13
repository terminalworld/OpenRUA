import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
r = Robot("iktest3")
def Q(z, y):
    z = np.array(z, float); y = np.array(y, float); x = np.cross(y, z)
    return Rot.from_matrix(np.stack([x, y, z], 1)).as_quat()
sl = [0, 0.866, 0.5]
Qa = Q([1,0,0], sl)
Qb = Q([0,0,-1], [0.5,0.866,0])
cases = {
 "pre (-0.335,0.118,0.954)": ((-0.335,0.118,0.954), Qa),
 "grasp (-0.215,0.118,0.954)": ((-0.215,0.118,0.954), Qa),
 "lift (-0.215,0.118,1.05)": ((-0.215,0.118,1.05), Qa),
 "lift2 (-0.215,0.118,1.15)": ((-0.215,0.118,1.15), Qa),
 "rot down (-0.30,0.0,1.25)": ((-0.30,0.0,1.25), Qb),
 "perch (-0.434,-0.041,1.15)": ((-0.434,-0.041,1.15), Qb),
}
seeds = [r.arm_q(), [0,-0.785,0,-2.356,0,1.571,0.785], [-0.5,0.3,0.2,-2.0,0,2.3,0.5], [0.5,0.5,-0.5,-1.8,0.3,2.4,1.0], [0.06,0.8,-0.43,-2.23,2.9,1.65,-0.45], [1.93,-0.96,-2.44,-2.15,2.41,2.07,-1.09]]
for name,(p,q) in cases.items():
    sols=[]
    for s in seeds:
        sol = r.ik(p, q, seed=s, timeout=0.5)
        if sol: sols.append(np.round(sol,2))
    print(name, "->", len(sols), "sols"); [print("    ", s) for s in sols[:3]]
