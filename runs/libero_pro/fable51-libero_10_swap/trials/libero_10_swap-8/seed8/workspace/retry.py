import numpy as np, time
from rob import *
r=Rob("retry"); sols=np.load("sols.npy",allow_pickle=True).item()
q=sols["A_pre_high"]
for i in range(3):
    t0=time.time(); code,err=r.move_q(q,4.0); print("wall",round(time.time()-t0,1),"q",r.arm_q().round(3))
    if code==0: break
