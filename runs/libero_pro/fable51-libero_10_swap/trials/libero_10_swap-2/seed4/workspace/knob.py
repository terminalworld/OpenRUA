from rk import *
m = Mover("knob")
KX, KY = -0.205, 0.204
print("open gripper"); m.gripper(0.04)
print("pre-grasp above knob"); m.goto_pose([KX, KY, 1.14], TOPDOWN, 4.0)
print("descend to ridge"); m.goto_pose([KX, KY, 1.038], TOPDOWN, 2.0, max_delta=0.6)
print("close on ridge"); m.gripper(0.0)
q = m.arm_q(); print("q before turn", np.round(q, 3))
