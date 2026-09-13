from ctl import *
c = Ctl()
c.move([0.131,0.035,0.72], DOWN, secs=4.0); print("fingers", np.round(c.fingers(),4))
c.move([0.131,0.035,0.585], DOWN, secs=3.0); print("fingers", np.round(c.fingers(),4))
c.gripper(0.04)
c.move([0.131,0.035,0.75], DOWN, secs=3.0)
