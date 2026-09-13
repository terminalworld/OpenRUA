from arm import *
a=Arm()
Q=yaw_quat(45)
T=np.array([-0.037,0.225])
print("fingers",a.fingers(),flush=True)
a.go([-0.15,-0.184,0.75],yaw_quat(14),secs=2.5)
a.go([-0.10,0.02,0.75],yaw_quat(30),secs=3)
a.go([T[0],T[1],0.75],Q,secs=3)
print("fingers over basket",a.fingers(),flush=True)
