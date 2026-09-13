import numpy as np, cv2
for cam in ["sideview","frontview","birdview"]:
    P=np.load(f"snaps/{cam}_world.npy"); img=cv2.imread(f"snaps/{cam}.png")
    X,Y,Z=P[...,0],P[...,1],P[...,2]
    r=np.hypot(X+0.16,Y-0.04)
    m=(r<0.14)&(Z>0.905)&(Z<1.1)&np.isfinite(Z)
    if m.sum()==0: print(cam,"none"); continue
    print(cam, m.sum(), "x",X[m].min().round(3),X[m].max().round(3),"y",Y[m].min().round(3),Y[m].max().round(3),"z",Z[m].min().round(3),Z[m].max().round(3), "centroid", X[m].mean().round(4), Y[m].mean().round(4))
    top=m&(Z>Z[m].max()-0.008)
    print(" rim", X[top].min().round(3),X[top].max().round(3),Y[top].min().round(3),Y[top].max().round(3), "centroid", X[top].mean().round(4), Y[top].mean().round(4), top.sum())
    vis=img.copy(); vis[m]=(0,0,255); cv2.imwrite(f"snaps/{cam}_bowlmask.png", vis)
