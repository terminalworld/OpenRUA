import sys, numpy as np
from arm import Arm, down_quat, world_to_base, base_to_world
a = Arm("iktest2")
q0 = a.arm_q()
p, quat = a.fk(q0)
print("fk hand", p.round(4), quat.round(4))
q = a.ik(p, quat, seed=q0, at_tcp=False)
print("ik at current hand pose:", None if q is None else q.round(3))
# slightly lower
q = a.ik(p - [0,0,0.1], quat, seed=q0, at_tcp=False)
print("ik 10cm lower:", None if q is None else q.round(3))
q = a.ik([0.3,0,0.5], down_quat(0), seed=q0, at_tcp=False)
print("ik std pose:", None if q is None else q.round(3))
q = a.ik([0.3,0,0.5], quat, seed=q0, at_tcp=False)
print("ik std pose cur quat:", None if q is None else q.round(3))
