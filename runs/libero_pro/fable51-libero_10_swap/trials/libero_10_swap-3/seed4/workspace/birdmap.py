import numpy as np
d=np.load('bird_depth.npy'); fx=fy=579.4112549695428; cx=320; cy=240
v,u=np.mgrid[0:480,0:640]
Xc=(u-cx)*d/fx; Yc=(v-cy)*d/fy
wx=Yc-0.2; wy=Xc; wz=3.0-d
np.save('bird_wx.npy',wx); np.save('bird_wy.npy',wy); np.save('bird_wz.npy',wz)
def report(name,mask):
    if mask.sum()==0: print(name,'empty'); return
    print(f"{name}: n={mask.sum()} x[{wx[mask].min():.3f},{wx[mask].max():.3f}] y[{wy[mask].min():.3f},{wy[mask].max():.3f}] z[{wz[mask].min():.3f},{wz[mask].max():.3f}] centroid x={wx[mask].mean():.3f} y={wy[mask].mean():.3f}")
# bowl: near (340,250) region, z 0.93-1.0
reg=(abs(u-340)<45)&(abs(v-250)<45)
report('bowl rim', reg&(wz>0.95)&(wz<1.0))
report('bowl any', reg&(wz>0.905)&(wz<1.0))
# cabinet region: u<310, v 200-330
creg=(u<310)&(v>190)&(v<340)
report('cabinet top >1.1', creg&(wz>1.1))
report('cabinet 1.0-1.1', creg&(wz>1.0)&(wz<1.1))
report('drawer front 0.95-1.0', creg&(wz>0.95)&(wz<1.0))
report('drawer bottom 0.91-0.95', creg&(wz>0.91)&(wz<0.95))
# print z histogram in cabinet region
zs=wz[creg]; h,e=np.histogram(zs,bins=np.arange(0.89,1.2,0.01))
for hh,ee in zip(h,e): 
    if hh: print(f"{ee:.2f} {hh}")
