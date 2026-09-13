import numpy as np, cv2
P=np.load("snaps/agentview_world.npy")
img=cv2.imread("snaps/agentview.png")
X,Y,Z=P[...,0],P[...,1],P[...,2]
r=np.hypot(X+0.17,Y-0.05)
m=(r<0.13)&(Z>0.905)&(Z<1.1)
print("bowl", m.sum(), "x",X[m].min().round(3),X[m].max().round(3),"y",Y[m].min().round(3),Y[m].max().round(3),"z",Z[m].min().round(3),Z[m].max().round(3), "centroid", X[m].mean().round(4), Y[m].mean().round(4))
top=m&(Z>Z[m].max()-0.008)
print(" rim", X[top].min().round(3),X[top].max().round(3),Y[top].min().round(3),Y[top].max().round(3), "centroid", X[top].mean().round(4), Y[top].mean().round(4), top.sum())
vis=img.copy(); vis[m]=(0,0,255); cv2.imwrite("snaps/bowlmask.png", vis)
# drawer interior: points with z in 0.915..0.94, y<-0.1
m=(Y<-0.1)&(Y>-0.5)&(X>-0.4)&(X<0.2)&(Z>0.91)&(Z<0.945)
print("drawer floor", m.sum(), "x",X[m].min().round(3),X[m].max().round(3),"y",Y[m].min().round(3),Y[m].max().round(3),"z",Z[m].mean().round(4))
for zlo,zhi in [(0.945,1.0),(1.0,1.05),(1.05,1.1),(1.1,1.15),(1.15,1.25)]:
    mm=(Y<-0.05)&(Y>-0.5)&(X>-0.4)&(X<0.2)&(Z>=zlo)&(Z<zhi)
    if mm.sum(): print(f" z{zlo}-{zhi}: n={mm.sum()} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}]")
