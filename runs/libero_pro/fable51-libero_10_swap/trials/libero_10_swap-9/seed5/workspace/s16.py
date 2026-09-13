import numpy as np, sys
import scene
D,C,K,R,t=scene.cam_model('robot0_eye_in_hand')
P=scene.to_world(D,K,R,t)
X,Y,Z=P[...,0],P[...,1],P[...,2]
ok=np.isfinite(Z)
m=ok&(np.abs(X-0.009)<0.09)&(np.abs(Y-0.007)<0.09)&(Z>0.905)&(Z<1.1)
print("pts",m.sum(),"zmax",Z[m].max())
for zt in [0.92,0.94,0.95,0.96,0.965,0.97,0.975,0.98,0.985,0.99,1.0]:
    mm=m&(Z>zt)
    if mm.sum(): print(zt,mm.sum(),"x",np.round([X[mm].min(),X[mm].max()],3),"y",np.round([Y[mm].min(),Y[mm].max()],3),"c",np.round([X[mm].mean(),Y[mm].mean()],3))
# table height near mug
tb=ok&(np.abs(X-0.009)<0.15)&(np.abs(Y-0.007)<0.15)&(Z<0.92)
print("table z", np.round(np.median(Z[tb]),4))
import cv2; cv2.imwrite('snaps/eih_top.png',C)
