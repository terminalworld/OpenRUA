import numpy as np
d=np.load('birdview_depth.npy')
fx=fy=579.4112549695428; cx=320; cy=240
# camera at (-0.2, 0, 3.0), image right=+y, image down=+x
def px2w(u,v,z=None):
    if z is None: z=d[v,u]
    X=(u-cx)*z/fx; Y=(v-cy)*z/fy
    return np.array([-0.2+Y, 0.0+X, 3.0-z])
print(d.shape)
tab=d[300,300]; print('table z', 3.0-tab)
# cup
sub=d[230:300,290:370]
mask=sub<tab-0.03
ys,xs=np.nonzero(mask)
print('cup px x',xs.min()+290,xs.max()+290,'y',ys.min()+230,ys.max()+230, 'top z', 3.0-sub[mask].min())
for r in range(0,70,2):
    print(''.join('#' if mask[r,c] else '.' for c in range(0,80,1)))
# Cup rim: pixels with depth near the min (rim top)
rim = sub < sub[mask].min()+0.01
ys,xs=np.nonzero(rim)
u=xs.mean()+290; v=ys.mean()+230
print('rim center px',u,v,'world',px2w(int(u),int(v),sub[mask].min()))
print('rim px extents', xs.min()+290, xs.max()+290, ys.min()+230, ys.max()+230)
