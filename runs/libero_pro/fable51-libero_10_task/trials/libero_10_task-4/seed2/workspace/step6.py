from ctl import *
c=Ctl()
print("hover yellow", c.move_tcp((-0.03,0.105,0.70), yaw=0.0, seconds=3.0))
c.close()
