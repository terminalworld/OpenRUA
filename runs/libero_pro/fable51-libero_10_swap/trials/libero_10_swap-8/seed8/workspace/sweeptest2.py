import numpy as np
from sweep import *
for sgn in (1,-1):
    for t in (0,15,30):
        for x in (0.10,0.14,0.18):
            t_=np.radians(t); hz=np.array([-np.sin(t_),0,-np.cos(t_)]); hy=sgn*np.array([np.cos(t_),0,-np.sin(t_)])
            R=R_from_axes(hz,hy); p=np.array([x,-0.05,0.96]); q=ik(p,R)
            print(f"sgn={sgn} t={t} x={x}: {'FAIL' if q is None else q.round(2)}",flush=True)
