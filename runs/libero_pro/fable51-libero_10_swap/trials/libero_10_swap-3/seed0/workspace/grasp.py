from robot import *
r = Robot("grasp")
print("close:", r.gripper(0.0))
print("fingers", r.fingers(), "wrench", np.round(r.wrench()[0],3))
