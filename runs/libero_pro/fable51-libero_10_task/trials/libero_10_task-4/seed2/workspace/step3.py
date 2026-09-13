from ctl import *
c=Ctl()
ok=c.move_tcp((-0.078,-0.187,0.66), yaw=0.0, seconds=2.5); print("lift ok",ok)
print("fingers", c.fingers())
c.close()
