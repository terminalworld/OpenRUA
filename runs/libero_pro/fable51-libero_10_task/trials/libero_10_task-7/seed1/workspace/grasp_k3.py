from arm import *
a=Arm()
Q=yaw_quat(14)
G=np.array([-0.15,-0.184])
print("-> descend",flush=True); a.go([G[0],G[1],0.52],Q,secs=3); a.go([G[0],G[1],0.45],Q,secs=3)
a.gripper(0.0)
print("fingers after close",a.fingers(),flush=True)
print("-> lift",flush=True); a.go([G[0],G[1],0.70],Q,secs=3)
print("fingers after lift",a.fingers(),flush=True)
