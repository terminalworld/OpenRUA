from ctl import *
c=Ctl()
c.gripper(0.04)
print("retreat", c.move_tcp((-0.006,0.261,0.72), yaw=0.0, seconds=2.5))
c.close()
