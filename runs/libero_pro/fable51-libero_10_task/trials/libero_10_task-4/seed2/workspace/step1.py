from ctl import *
c=Ctl()
ok=c.move_tcp((-0.078,-0.187,0.65), yaw=0.0, seconds=3.0)
print("hover ok",ok)
c.close()
