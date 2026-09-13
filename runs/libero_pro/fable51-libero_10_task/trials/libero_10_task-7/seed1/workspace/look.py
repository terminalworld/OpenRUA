from arm import *
a=Arm()
a.gripper(0.04)
for i in range(3):
    code,err=a.move_tcp_world([-0.11,-0.17,0.72],DOWN,secs=4)
    if code==0 and err<0.01: break
p,q=a.hand_pose_world(); print("hand",p.round(4),np.round(q,3),flush=True)
