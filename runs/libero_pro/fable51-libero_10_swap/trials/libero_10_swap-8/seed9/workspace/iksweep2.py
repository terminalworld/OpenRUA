import numpy as np, time, sys
from rob import *
r = Robot("iksweep2")
def rot_y(th):
    c,s=np.cos(th),np.sin(th); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
Rfx = quat_to_R(0.7071,-0.7071,0,0)   # fingers along x, z down
seeds = [np.array([0.08,1.23,-0.07,-0.72,0.08,1.94,1.6]), np.array([0.0,0.9,0.0,-1.0,0.0,1.9,0.8]), np.array([0.0,1.4,0.0,-0.4,0.0,1.8,0.8])]
for pitch in [0, -10]:
    R = rot_y(np.radians(pitch)) @ Rfx; q = R_to_quat(R)
    for y in [-0.01, 0.09]:
        for x in [0.15,0.16,0.17,0.18,0.19]:
            got=None
            for s in seeds:
                sol = r.ik_tcp_world((x,y,1.0), q, seed=s, timeout=4.0)
                if sol is not None: got=sol; break
            print(f"pitch={pitch} y={y} x={x}: {'OK '+str(np.round(got,2)) if got is not None else 'fail'}", flush=True)
            if got is None: break
