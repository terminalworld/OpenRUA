import numpy as np, itertools
from rob import *
r=Rob("iktest")
seed=r.arm_q()
def hR(hz,hy): return R_from_axes(hz,hy)
tests=[]
for x in (0.10,0.13,0.16,0.20):
  for z in (1.00,1.08,1.16,1.24):
    for tilt in (0,20,35):
      th=np.radians(tilt); hz=np.array([np.sin(th),0,-np.cos(th)])
      for hyaz in (90,):
        hy=np.array([0,1.0,0])
        q=r.ik([x,0.05,z],hR(hz,hy),seed=seed,at_tcp=True)
        print(f"x={x} z={z} tilt={tilt}: {'ok '+str(q.round(2)) if q is not None else 'FAIL'}")
