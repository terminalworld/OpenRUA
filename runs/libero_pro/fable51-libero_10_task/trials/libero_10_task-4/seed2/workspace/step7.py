from ctl import *
c=Ctl()
GX,GY=-0.030,0.148
print("hover", c.move_tcp((GX,GY,0.66), yaw=0.0, seconds=2.5))
print("descend", c.move_tcp((GX,GY,0.517), yaw=0.0, seconds=2.5))
f=c.gripper(0.0)
print("lift", c.move_tcp((GX,GY,0.66), yaw=0.0, seconds=2.5))
print("fingers after lift", c.fingers())
c.close()
