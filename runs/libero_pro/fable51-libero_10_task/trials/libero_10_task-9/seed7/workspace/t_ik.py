from rlib import *
import rlib
r = Robot()
q0 = r.joints()
pos, quat = r.tf("world","panda_hand")
rlib.BASE = np.zeros(3)   # try raw world coords
s = r.ik_world(pos, quat, seed=q0, attempts=1)
print("IK(world coords):", None if s is None else np.round(s,3))
rlib.BASE = np.array([-0.66,0,0.912])
s2 = r.ik_world(pos, quat, seed=q0, attempts=1)
print("IK(base coords):", None if s2 is None else np.round(s2,3))
print("current:", np.round(q0,3))
