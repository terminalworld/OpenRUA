from arm import *
a=Arm()
K=np.array([-0.200,-0.150])
def go(z,secs):
    for i in range(3):
        code,err=a.move_tcp_world([K[0],K[1],z],DOWN_X,secs=secs)
        if code==0 and err<0.01: break
    p,q=a.hand_pose_world(); print("hand",p.round(4),"tcp z",round(p[2]-TCP,4),np.round(q,3),flush=True)
print("-> descend",flush=True); go(0.56,3); go(0.475,3)
a.gripper(0.0)
f=a.fingers(); print("fingers after close",f,flush=True)
print("-> lift",flush=True); go(0.70,3)
print("fingers after lift",a.fingers(),flush=True)
