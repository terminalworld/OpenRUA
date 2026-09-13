from arm import *
a=Arm()
Q=yaw_quat(45)
T=np.array([-0.037,0.225])
a.go([T[0],T[1],0.66],Q,secs=2.5)
print("fingers before release",a.fingers(),flush=True)
a.gripper(0.04)
a.go([T[0],T[1],0.78],Q,secs=2.5)
print("fingers after",a.fingers(),flush=True)
