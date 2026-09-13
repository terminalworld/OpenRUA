from lib import *
r = Robot("m1")
q = r.move_tcp((-0.206,-0.195,1.25), down_quat(90), seconds=3)
print("q", np.round(r.arm_q(),3))
