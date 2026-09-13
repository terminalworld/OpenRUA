from ctl import *
c = Ctl()
c.move([-0.037,0.091,0.50], DOWN, secs=2.5)
c.move([-0.037,0.091,0.443], DOWN, secs=2.5)
c.gripper(0.0)
c.move([-0.037,0.091,0.65], DOWN, secs=3.0); print("fingers", np.round(c.fingers(),4))
