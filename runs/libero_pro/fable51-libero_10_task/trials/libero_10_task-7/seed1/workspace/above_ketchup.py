from arm import *
a=Arm()
K=np.array([-0.206,-0.148])
print("-> above ketchup",flush=True)
a.move_tcp_world([K[0],K[1],0.70],DOWN,secs=3.5)
p,q=a.hand_pose_world(); print("hand",p.round(4),np.round(q,3),"fingers",a.fingers(),flush=True)
