from ctl import *
c=Ctl()
# mug centre sits +0.046 y from the TCP (grasped on the -y wall)
PLATE=(-0.006,0.307); off=0.046
tx,ty=PLATE[0],PLATE[1]-off
print("raise", c.move_tcp((-0.078,-0.187,0.75), yaw=0.0, seconds=2.0))
print("over plate", c.move_tcp((tx,ty,0.75), yaw=0.0, seconds=3.5))
print("fingers", c.fingers())
print("lower", c.move_tcp((tx,ty,0.55), yaw=0.0, seconds=3.0))
print("fingers", c.fingers())
c.close()
