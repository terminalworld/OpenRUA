from rk import *
m = Mover("knob3")
print("open"); m.gripper(0.04)
p,qq = m.fk_world(); print("hand", np.round(p,4), np.round(qq,3))
print("lift"); m.goto_pose([p[0], p[1], 1.20], qq, 2.5, max_delta=0.8)
