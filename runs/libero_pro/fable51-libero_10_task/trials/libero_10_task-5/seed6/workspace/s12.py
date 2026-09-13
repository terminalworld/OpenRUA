import numpy as np
from ctl import Robot
from wr import Wrench
r = Robot("s12"); w = Wrench(r)
print("before", w.get(), r.finger())
f = r.gripper(0.0)
print("after", w.get(), r.hand_pose()[0].round(4))
r.snap("robot0_eye_in_hand","/workspace/s12_eih.png"); r.snap("sideview","/workspace/s12_side.png")
