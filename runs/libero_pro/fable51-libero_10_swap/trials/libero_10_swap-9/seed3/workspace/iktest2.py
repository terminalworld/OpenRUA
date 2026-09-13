import numpy as np, rob, sys
from scipy.spatial.transform import Rotation as Rot
r=rob.Robot()
lo=np.array([-2.9,-1.76,-2.9,-3.07,-2.9,-0.02,-2.9]); hi=np.array([2.9,1.76,2.9,-0.07,2.9,3.75,2.9]); rng=np.random.default_rng(4)
for th in (15,20,25,30):
    t=np.radians(th); zh=np.array([0,np.cos(t),-np.sin(t)]); xh=np.array([0,-np.sin(t),-np.cos(t)]); yh=np.cross(zh,xh)
    qt=list(Rot.from_matrix(np.stack([xh,yh,zh],1)).as_quat())
    for x in (-0.14,):
      for y in (-0.52,-0.50,-0.47,-0.44,-0.42):
        zo=1.08+0.1034*np.sin(t); pos=[x,y,zo]; sols=[]
        for k in range(12):
            try: qq=r.ik(pos,qt,seed=list(rng.uniform(lo,hi)),timeout=0.4,avoid=False)
            except Exception: qq=None
            if qq is not None: sols.append(np.round(qq,2))
        print(th,np.round(pos,3),len(sols),sols[0] if sols else "",flush=True)
