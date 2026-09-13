import numpy as np, rclpy, sys, itertools
from rob import *
rclpy.init(); r=Robot()
def Rtilt(t,flip):
    t=np.radians(t); z=[0,np.cos(t),-np.sin(t)]
    x=[0,np.sin(t),np.cos(t)] if not flip else [0,-np.sin(t),-np.cos(t)]
    return R_from_axes(z,x)
found=None
for t,flip,x,y,z in itertools.product([0,25,45],[False,True],[-0.30,-0.20,-0.12],[-0.62,-0.55,-0.50],[1.05,1.20,1.35]):
    R=Rtilt(t,flip); p=[x,y,z]
    q=best_ik(r,p,R,at_tcp=False,timeout=1.5)
    if q is not None:
        print('OK',t,flip,p); found=(p,R)
        if y<=-0.55: break
if found:
    p,R=found; ok=goto(r,p,R,at_tcp=False,seconds=4); print('goto ok',ok)
    r.snap('eih','/workspace/eih_door.png'); print('tcp',r.tcp()[0])
