import numpy as np, sys
d=np.load(sys.argv[1] if len(sys.argv)>1 else "pr_bird_depth.npy")
fx=579.4112549695428; cx=320; cy=240
v,u=np.mgrid[0:480,0:640]
X=(u-cx)*d/fx; Y=(v-cy)*d/fx
wx=Y-0.2; wy=X; wz=3.0-d
m=(wx>-0.13)&(wx<0.13)&(wy>0.05)&(wy<0.27)
sel=m&(wz>0.93)&(wz<1.0)
pts=np.stack([wx[sel],wy[sel],wz[sel]],1)
inside=(pts[:,0]>-0.10)&(pts[:,0]<0.092)&(pts[:,1]>0.10)&(pts[:,1]<0.248)
b=pts[inside]
print("bottle pts",len(b))
print("x range",b[:,0].min(),b[:,0].max(),"y range",b[:,1].min(),b[:,1].max(),"z max",b[:,2].max())
A=np.array([-0.107,0.095]); C=np.array([0.099,0.254]); dh=(C-A)/np.linalg.norm(C-A)
s=(b[:,:2]-A)@dh
for lo in np.arange(0,0.27,0.02):
    k=(s>=lo)&(s<lo+0.02)
    if k.sum(): print(f"s {lo:.2f}: n={k.sum()} zmax={b[k,2].max():.3f} zmed={np.median(b[k,2]):.3f}")
hi=m&(wz>0.99)&(wz<1.12)
print("pts above 0.99 in drawer region:",hi.sum())
if hi.sum(): print(" x",wx[hi].min(),wx[hi].max()," y",wy[hi].min(),wy[hi].max()," z",wz[hi].max())
# drawer front panel location: top of panel z~0.984 at y ~0.087-0.095
pan=(wx>-0.08)&(wx<0.08)&(wy>0.0)&(wy<0.12)&(wz>0.975)&(wz<0.995)
print("front panel y range", wy[pan].min() if pan.sum() else None, wy[pan].max() if pan.sum() else None)
