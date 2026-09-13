import numpy as np, sys
npy = sys.argv[1] if len(sys.argv)>1 else 'snaps/bird_depth4.npy'
d=np.load(npy)
fx=fy=579.4112549695428; cx=320; cy=240
H,W=d.shape
vv,uu=np.mgrid[0:H,0:W]
X=(vv-cy)*d/fy - 0.2; Y=(uu-cx)*d/fx; Z=3.0-d
# book candidates: Z in 0.95..1.2, X in -0.3..0.2, Y in -0.15..0.25 (exclude caddy, robot)
m=(Z>0.95)&(Z<1.2)&(X>-0.3)&(X<0.2)&(Y>-0.15)&(Y<0.25)
# remove mug region (mug near -0.2, 0.14) -> mug Z max ~1.04
print('pts',m.sum())
for lo in [0.95,1.0,1.03,1.05,1.06,1.07]:
    mm=m&(Z>lo)
    if mm.sum()==0: continue
    pts=np.stack([X[mm],Y[mm]],1); c=pts-pts.mean(0); u,s,vt=np.linalg.svd(c,full_matrices=False)
    print(f'Z>{lo}: n={mm.sum()} center={pts.mean(0).round(4)} axis={vt[0].round(3)} along=({(c@vt[0]).min():.3f},{(c@vt[0]).max():.3f}) across=({(c@vt[1]).min():.3f},{(c@vt[1]).max():.3f}) Zmax={Z[mm].max():.3f}')
# check mug separately
mm=(Z>0.95)&(X>-0.3)&(X<-0.1)&(Y>0.08)&(Y<0.25)
print('mug-ish region n',mm.sum(), 'Zmax', Z[mm].max() if mm.sum() else None, 'center', (X[mm].mean(), Y[mm].mean()) if mm.sum() else None)
