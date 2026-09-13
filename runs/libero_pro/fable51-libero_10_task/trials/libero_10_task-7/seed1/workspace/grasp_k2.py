from arm import *
a=Arm()
Q=yaw_quat(14)
G=np.array([-0.15,-0.184])
print("fingers",a.fingers(),flush=True)
print("-> above",flush=True); p,q=a.go([G[0],G[1],0.60],Q,secs=4)
R=_qR(*q); print("hand Y axis (closing dir) world:",R[:,1].round(3),flush=True)
