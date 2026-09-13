from arm import *
a = Arm()
a.move_to((-0.30, -0.25, 0.95), Q_DOWN, seconds=3.0)
print("q:", np.round(a.arm_q(),3))
