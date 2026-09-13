import numpy as np
P = np.load("birdview_world.npy")
X,Y,Z = P[...,0],P[...,1],P[...,2]
def region(name, m):
    pts = P[m]
    print(name, "n=",len(pts), f"x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]")
    h,e = np.histogram(pts[:,2], bins=np.arange(0.9,1.5,0.02))
    print("  z hist:", [(f"{e[i]:.2f}",int(h[i])) for i in range(len(h)) if h[i]>0])
# white mug
region("white mug", (Z>0.93)&(X>-0.2)&(X<0)&(Y<-0.15)&(Y>-0.4))
region("yellow mug", (Z>0.93)&(X>-0.15)&(X<0.1)&(Y>-0.1)&(Y<0.1))
# microwave region: y > 0.05, x > -0.3
region("microwave+door", (Z>0.93)&(X>-0.3)&(Y>0.05))
# microwave body top
mw = (Z>1.09)&(Z<1.12)&(X>-0.3)&(Y>0.05)
region("mw top plane", mw)
door = (Z>1.09)&(Z<1.12)&(X>-0.3)&(Y>0.05)&(Y<0.29)
region("door top", door)
# door footprint: in the x=-0.207 plane; find all points with x in [-0.23,-0.18] and y in [0.05,0.3]
region("door slab", (Z>0.93)&(X>-0.24)&(X<-0.17)&(Y>0.05)&(Y<0.30))
# body
region("body", (Z>0.93)&(X>-0.3)&(Y>0.30))
# hand
region("hand", (Z>1.0)&(X>-0.35)&(X<-0.1)&(Y>-0.1)&(Y<0.1))
