import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot(); rng=np.random.default_rng(1)
lo=np.array([-2.9,-1.76,-2.9,-3.07,-2.9,-0.02,-2.9]); hi=np.array([2.9,1.76,2.9,-0.07,2.9,3.75,2.9])
def nsol(p,q,n=12):
    k=0;best=None
    for i in range(n):
        try: sol=r.ik(p,q,seed=list(rng.uniform(lo,hi)),avoid=False,timeout=0.25)
        except RuntimeError: sol=None
        if sol: k+=1; best=np.round(sol,2)
    return k,best
for ys in (1,-1):
  q=quat_from_axes(np.array([0,1.0,0]),np.array([0,0,float(ys)]))
  for x in (-0.20,-0.135):
    for y in (-0.55,-0.50,-0.45):
      for z in (1.03,1.15,1.30):
        k,b=nsol(np.array([x,y,z]),q); print(f"yh{ys:+d} x{x} y{y} z{z}: {k}/12 {b}")
rclpy.shutdown()
