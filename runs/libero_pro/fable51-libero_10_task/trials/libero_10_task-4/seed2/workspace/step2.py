from ctl import *
c=Ctl()
p,_=c.tcp_pose(); print("tcp before",np.round(p,4))
ok=c.move_tcp((-0.078,-0.187,0.518), yaw=0.0, seconds=2.5); print("descend ok",ok)
f=c.gripper(0.0)
print("fingers after close", f)
c.close()
