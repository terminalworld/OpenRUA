import numpy as np
from rob import *
r=Rob("iktest2"); seed=r.arm_q()
a=np.array([0,0.966,-0.259])   # pot A axis, lid->base
def tryp(name,p,hz,hy=None):
    hz=np.asarray(hz,float); hz/=np.linalg.norm(hz)
    if hy is None:
        hy=np.cross(a,hz); hy/=np.linalg.norm(hy)
        if hy[0]>0: hy=-hy
    R=R_from_axes(hz,hy); q=r.ik(p,R,seed=seed,at_tcp=True)
    fl=np.asarray(p)-TCP*hz
    print(f"{name}: hz={hz.round(2)} hy={R[:,1].round(2)} flange_r={np.hypot(fl[0]+0.66,fl[1]):.3f} -> {'ok '+str(q.round(2)) if q is not None else 'FAIL'}")
for tx in (0.2,0.3,0.4):
  for ty in (0.0,0.2,0.34):
    hz=np.array([tx,ty,-np.sqrt(max(1e-6,1-tx*tx-ty*ty))])
    tryp(f"lidA tx{tx} ty{ty}",[0.197,0.008,1.0],hz)
# hover above A for wrist cam, tilt 20 toward +x
tryp("hoverA",[0.16,0.03,1.22],[0.34,0,-0.94],[0,1,0])
tryp("hoverA2",[0.14,0.03,1.18],[0.34,0,-0.94],[0,1,0])
