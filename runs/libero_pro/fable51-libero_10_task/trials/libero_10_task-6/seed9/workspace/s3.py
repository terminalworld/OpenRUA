from ctl import *
c = Ctl()
c.move([-0.2255,0.035,0.72], DOWN, secs=3.0)
print("fingers", np.round(c.fingers(),4))
