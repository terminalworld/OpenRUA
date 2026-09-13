import numpy as np
d=np.load('birdview_depth.npy')
fx=fy=579.4112549695428; cx=320; cy=240
def px2w(u,v,z):
    X=(u-cx)*z/fx; Y=(v-cy)*z/fy
    return np.array([-0.2+Y, 0.0+X, 3.0-z])
tab=d[300,300]
y0,y1,x0,x1=248,282,300,352
sub=d[y0:y1,x0:x1]
mask=sub<tab-0.03
zmin=sub[mask].min(); print('cup top z', 3.0-zmin)
# body (exclude handle): depth near min
body = sub < zmin+0.008
ys,xs=np.nonzero(body)
u=xs.mean()+x0; v=ys.mean()+y0
print('body px center',u,v,'extent x',xs.min()+x0,xs.max()+x0,'y',ys.min()+y0,ys.max()+y0)
print('cup center world', px2w(u,v,zmin))
diam_px = xs.max()-xs.min()+1
print('diam m', diam_px*zmin/fx)
for r in range(0,y1-y0):
    print(''.join('#' if body[r,c] else ('+' if mask[r,c] else '.') for c in range(0,x1-x0)))
