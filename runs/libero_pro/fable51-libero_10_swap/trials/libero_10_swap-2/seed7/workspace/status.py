import numpy as np
from lib import Robot
r = Robot("status")
q = r.arm_q(); print("arm q:", np.round(q, 4))
p, quat, fid = r.fk_world(); print("hand world pos:", np.round(p,4), "quat:", np.round(quat,4), "fk frame:", fid)
print("finger gap:", round(r.finger_gap(),4)); print("wrench:", r.wrench())
