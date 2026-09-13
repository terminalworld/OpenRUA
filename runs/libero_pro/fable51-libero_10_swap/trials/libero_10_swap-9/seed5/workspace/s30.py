import numpy as np, scene
D,C,K,R,t=scene.cam_model('birdview')
P=scene.to_world(D,K,R,t); X,Y,Z=P[...,0],P[...,1],P[...,2]; ok=np.isfinite(Z)
m=ok&(X>-0.21)&(X<-0.06)&(Y>-0.50)&(Y<-0.28)&(Z>0.93)&(Z<1.3)
print("n",m.sum())
for lo,hi in [(0.93,0.97),(0.97,1.0),(1.0,1.03),(1.03,1.06),(1.06,1.09),(1.09,1.12),(1.12,1.16),(1.16,1.3)]:
    b=m&(Z>lo)&(Z<hi)
    if b.sum(): print(f"  z{lo}-{hi} n={b.sum()} x[{X[b].min():.3f},{X[b].max():.3f}] y[{Y[b].min():.3f},{Y[b].max():.3f}]")
import cv2; cv2.imwrite('snaps/bird_p.png',C)
