import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot(); rng=np.random.default_rng(2)
lo=np.array([-2.9,-1.76,-2.9,-3.07,-2.9,-0.02,-2.9]); hi=np.array([2.9,1.76,2.9,-0.07,2.9,3.75,2.9])
def nsol(p,q,n=10):
    k=0;best=None
    for i in range(n):
        try: sol=r.ik(p,q,seed=list(rng.uniform(lo,hi)),avoid=False,timeout=0.25)
        except RuntimeError: sol=None
        if sol: k+=1; best=np.round(sol,2)
    return k,best
for pitch in (0,20,35):
  th=np.radians(pitch); zh=np.array([0,np.cos(th),-np.sin(th)])
  for yh in ((1,0,0),(0,np.sin(th),np.cos(th))):
    q=quat_from_axes(zh,np.array(yh,float))
    for x in (-0.135,-0.17):
      for y in (-0.40,-0.45,-0.50):
        for z in (0.97,1.02):
          k,b=nsol(np.array([x,y,z]),q); print(f"pitch{pitch} yh{np.round(yh,2)} x{x} y{y} z{z}: {k}/10 {b}")
rclpy.shutdown()
