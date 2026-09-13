from arm import *
a=Arm()
K=np.array([-0.200,-0.148])
print("-> rotate above ketchup",flush=True)
for i in range(2):
    code,err=a.move_tcp_world([K[0],K[1],0.66],DOWN_X,secs=4)
    if code==0: break
p,q=a.hand_pose_world(); print("hand",p.round(4),np.round(q,3),"fingers",a.fingers(),flush=True)
