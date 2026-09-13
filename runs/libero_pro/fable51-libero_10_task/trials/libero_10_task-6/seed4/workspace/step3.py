from common import *
r = Robot()
f = r.gripper(0.0)
# settle a moment: hold pose briefly so the grasp state updates
q,_ = r.joints(); r.move([q], [0.5])
print("fingers after settle", r.fingers())
