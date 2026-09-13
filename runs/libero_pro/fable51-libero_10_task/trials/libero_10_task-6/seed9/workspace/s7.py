from ctl import *
c = Ctl()
c.move([0.13,0.15,0.65], DOWN, secs=4.0); print("fingers", np.round(c.fingers(),4))
c.move([0.13,0.15,0.449], DOWN, secs=3.0); print("fingers", np.round(c.fingers(),4))
c.gripper(0.04)
c.move([0.13,0.15,0.62], DOWN, secs=3.0)
c.move([-0.15,0.0,0.75], DOWN, secs=4.0)   # park clear of the scene for verification
