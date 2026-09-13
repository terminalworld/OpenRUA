from arm import *
a=Arm()
Q=yaw_quat(14)
G=np.array([-0.15,-0.184])
print("fingers",a.fingers(),flush=True)
for z in (0.50,0.56,0.62,0.70):
    try: a.go([G[0],G[1],z],Q,secs=2.5)
    except RuntimeError as e: print("z",z,e,flush=True)
print("fingers after lift",a.fingers(),flush=True)
