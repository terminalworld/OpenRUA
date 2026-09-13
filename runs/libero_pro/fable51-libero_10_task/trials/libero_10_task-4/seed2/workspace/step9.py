from ctl import *
c=Ctl()
print("park", c.move_tcp((-0.15,0.0,0.75), yaw=0.0, seconds=3.0))
c.close()
