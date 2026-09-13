from ctl import *
c=Ctl()
PLATE=(-0.013,-0.281); off=0.046   # mug centre sits -0.046 y from the TCP (grasped on +y wall)
tx,ty=PLATE[0],PLATE[1]+off
print("raise", c.move_tcp((-0.03,0.148,0.75), yaw=0.0, seconds=2.0))
print("over plate", c.move_tcp((tx,ty,0.75), yaw=0.0, seconds=3.5))
print("fingers", c.fingers())
print("lower", c.move_tcp((tx,ty,0.55), yaw=0.0, seconds=3.0))
print("fingers", c.fingers())
c.gripper(0.04)
print("retreat", c.move_tcp((tx,ty,0.72), yaw=0.0, seconds=2.5))
c.close()
