import numpy as np
d=np.load("/workspace/birdview_depth.npy"); fx=579.4112549695428; cx=320; cy=240
H,W=d.shape; uu,vv=np.meshgrid(np.arange(W),np.arange(H))
wx=(vv-cy)*d/fx-0.2; wy=(uu-cx)*d/fx; wz=3.0-d
# region around the bottle/hand xy (-0.137,0.049): +-0.12
m=(np.abs(wx+0.137)<0.12)&(np.abs(wy-0.049)<0.12)
for lo in np.arange(0.90,1.5,0.02):
    mm=m&(wz>=lo)&(wz<lo+0.02)
    if mm.sum()>3: print(f"z[{lo:.2f}] n={mm.sum():4d} x[{wx[mm].min():.3f},{wx[mm].max():.3f}] y[{wy[mm].min():.3f},{wy[mm].max():.3f}]")
from rob import Robot
r=Robot("hc"); p,q,R=r.tcp(); ph,_,_=r.fk_hand()
print("reported tcp",np.round(p,4),"hand",np.round(ph,4))
