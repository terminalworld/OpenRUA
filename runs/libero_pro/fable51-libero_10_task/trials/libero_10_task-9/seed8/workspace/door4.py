import numpy as np
d = np.load("agentview_cloud.npz"); pw = d["pw"]
X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
m = (Z>0.95)&(Z<1.10)&(X<-0.18)&(X>-0.5)&(Y>-0.1)&(Y<0.30)
P = np.c_[X[m],Y[m]]
c = P.mean(0); u,s,vt = np.linalg.svd(P-c, full_matrices=False)
dirv = vt[0]; 
if dirv[0] > 0: dirv = -dirv   # point from hinge toward tip (toward -x)
t = (P-c)@dirv; perp=(P-c)@vt[1]
print("centroid", c.round(3), "dir", dirv.round(3), "angle", np.degrees(np.arctan2(dirv[1],dirv[0])).round(1))
print("t range", np.percentile(t,[0.5,99.5]).round(3), "perp spread", np.percentile(perp,[0.5,99.5]).round(3))
hinge_end = c + dirv*np.percentile(t,0.5); tip_end = c + dirv*np.percentile(t,99.5)
print("hinge end", hinge_end.round(3), "tip end", tip_end.round(3), "length", (np.percentile(t,99.5)-np.percentile(t,0.5)).round(3))
# frontview too
d = np.load("frontview_cloud.npz"); pw = d["pw"]
X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
m = (Z>0.95)&(Z<1.10)&(X<-0.18)&(X>-0.5)&(Y>-0.1)&(Y<0.30)
P = np.c_[X[m],Y[m]]
t=(P-c)@dirv; print("frontview t range", np.percentile(t,[0.5,99.5]).round(3), "perp", np.percentile((P-c)@vt[1],[0.5,99.5]).round(3))
