from arm import *
a=Arm()
BOX=np.array([-0.137,0.056]); TOP=0.455
print("state",np.round(a.joints(),3),a.fingers(),flush=True)
a.gripper(0.04)
print("-> above box",flush=True)
a.move_tcp_world([BOX[0],BOX[1],TOP+0.10],DOWN,secs=3)
p,q=a.hand_pose_world(); print("hand",p.round(4),np.round(q,4),flush=True)
