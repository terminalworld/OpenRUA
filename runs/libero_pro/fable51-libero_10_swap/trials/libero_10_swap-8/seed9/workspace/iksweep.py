import numpy as np, time, sys
from rob import *
r = Robot("iksweep")
def rot_y(th):
    c,s=np.cos(th),np.sin(th); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
Rfx = quat_to_R(0.7071,-0.7071,0,0)   # fingers along x, z down
Rfy = quat_to_R(1,0,0,0)             # fingers along y, z down
seed = np.array([0.0, 0.4, 0.0, -1.8, 0.0, 2.2, 0.8])
z = 1.0
for pitch in [-20, -35, -50]:
    for name,Rb in [("fx",Rfx),("fy",Rfy)]:
        R = rot_y(np.radians(pitch)) @ Rb
        q = R_to_quat(R)
        reach = None
        for x in [0.10,0.14,0.17,0.20,0.24]:
            t=time.time()
            sol = r.ik_tcp_world((x,0.04,z), q, seed=seed)
            print(f"   x={x} {'ok' if sol is not None else 'fail'} {time.time()-t:.1f}s", flush=True)
            if sol is None: break
            reach = x; last=sol
        print(f"z={z} pitch={pitch} {name}: max x reachable = {reach}", "" if reach is None else np.round(last,2), flush=True)
