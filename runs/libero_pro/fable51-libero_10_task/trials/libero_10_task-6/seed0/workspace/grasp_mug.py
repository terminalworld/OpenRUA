from robot import *
r = Robot("grasp")
print("fingers before", np.round(r.fingers(),4), "wrench", np.round(r.wrench(),2))
r.gripper(0.0)
f = r.fingers(); print("fingers after close", np.round(f,4), " gap=", round(abs(f[0])+abs(f[1]),4))
print("wrench", np.round(r.wrench(),2))
