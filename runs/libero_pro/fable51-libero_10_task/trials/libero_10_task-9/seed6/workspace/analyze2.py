import numpy as np
P = np.load("birdview_world.npy")
X,Y,Z = P[...,0],P[...,1],P[...,2]
top = (Z>1.09)&(Z<1.12)&(X>-0.15)&(Y>0.1)
pts = P[top]
print("body top: x[%.3f,%.3f] y[%.3f,%.3f]"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max()))
# per-row of x, min/max y (to see shape)
for x0 in np.arange(-0.2, 0.2, 0.02):
    m = top & (X>=x0)&(X<x0+0.02)
    if m.sum(): print(f"x={x0:.2f}: y[{Y[m].min():.3f},{Y[m].max():.3f}] n={m.sum()}")
# door slab: z in [1.09,1.12], x<-0.15, y in [0,0.3]
door = (Z>1.09)&(Z<1.12)&(X<-0.15)&(Y>0.0)&(Y<0.30)
pts = P[door]
print("door top: x[%.3f,%.3f] y[%.3f,%.3f] n=%d"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),len(pts)))
# fit line to door points
A = np.c_[pts[:,1], np.ones(len(pts))]
k,b = np.linalg.lstsq(A, pts[:,0], rcond=None)[0]
print(f"door line: x = {k:.3f}*y + {b:.3f}; angle from y axis = {np.degrees(np.arctan(k)):.1f} deg")
for y0 in np.arange(0.0, 0.32, 0.02):
    m = door & (Y>=y0)&(Y<y0+0.02)
    if m.sum(): print(f"y={y0:.2f}: x[{X[m].min():.3f},{X[m].max():.3f}] n={m.sum()}")
