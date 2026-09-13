import numpy as np
d=np.load('snaps/bird_depth.npy')
fx=fy=579.4112549695428; cx=320; cy=240
H,W=d.shape
vv,uu=np.mgrid[0:H,0:W]
X=(vv-cy)*d/fy - 0.2   # world x  (image down)
Y=(uu-cx)*d/fx         # world y  (image right)
Z=3.0-d
# objects above table
mask=(Z>0.885)&(X>-0.6)&(X<0.4)&(np.abs(Y)<0.6)
# exclude robot: robot occupies X<-0.3ish and Y in (-0.1,0.2) at high Z; print clusters by Z
import collections
# book: region around (325,272)
bm=mask&(uu>300)&(uu<350)&(vv>255)&(vv<295)
print('book pts',bm.sum(),'X',X[bm].min().round(3),X[bm].max().round(3),'Y',Y[bm].min().round(3),Y[bm].max().round(3),'Z',Z[bm].min().round(3),Z[bm].max().round(3))
# book top-surface points (Z>1.0)
bt=bm&(Z>1.0)
pts=np.stack([X[bt],Y[bt]],1); print('book top pts',len(pts), 'center',pts.mean(0).round(4))
# PCA for orientation
c=pts-pts.mean(0); u,s,vt=np.linalg.svd(c,full_matrices=False); print('book axis',vt[0].round(3),'extent along',(c@vt[0]).min().round(3),(c@vt[0]).max().round(3),'across',(c@vt[1]).min().round(3),(c@vt[1]).max().round(3))
# caddy: region uu 210-305, vv 155-210
cm=mask&(uu>205)&(uu<310)&(vv>150)&(vv<215)
print('caddy Z range',Z[cm].min().round(3),Z[cm].max().round(3))
# walls: Z > 0.95
wm=cm&(Z>0.95)
print('caddy walls X',X[wm].min().round(3),X[wm].max().round(3),'Y',Y[wm].min().round(3),Y[wm].max().round(3))
# histogram of wall heights
hs,edges=np.histogram(Z[wm],bins=20); 
for h,e in zip(hs,edges): print(round(e,3),h)
# print an ascii map of Z over caddy region at 2px stride
for v in range(150,215,3):
    row=''
    for u in range(205,320,2):
        z=Z[v,u]
        row+= '#' if z>1.15 else ('W' if z>0.98 else ('w' if z>0.93 else ('f' if z>0.89 else '.')))
    print(v,row)
