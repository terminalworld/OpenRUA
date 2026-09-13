"""Grasp the stove knob bar from above and rotate it."""
import sys, numpy as np, pk

KNOB = np.array([-0.206, 0.195])
YAW = 0.0            # bar along world x -> fingers close along world y
Z_PRE, Z_GRASP = 1.03, 0.930
ROT = float(sys.argv[1]) if len(sys.argv) > 1 else np.pi / 2

r = pk.Robot("knob")
print("start q", np.round(r.joints(), 3), "fingers", r.fingers())
print("open gripper"); r.gripper(pk.GRIP["open_m"])
print("pre-grasp"); r.move_pose([*KNOB, Z_PRE], pk.topdown_R(YAW), 4.0)
print("descend"); r.move_pose([*KNOB, Z_GRASP], pk.topdown_R(YAW), 2.0)
print("close"); f = r.gripper(pk.GRIP["closed_m"])
print("wrench", r.wrench())
q = r.joints()
print("rotate j7 by", ROT)
q2 = q.copy(); q2[6] += ROT
r.move_joints([q2], 4.0)
print("after rot q", np.round(r.joints(), 3), "fingers", r.fingers(), "wrench", r.wrench())
print("DONE")
