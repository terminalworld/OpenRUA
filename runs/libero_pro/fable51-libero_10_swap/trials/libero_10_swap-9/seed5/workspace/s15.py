import numpy as np, subprocess, sys
sys.argv=['scene.py','birdview']
import scene
D,C,K,R,t=scene.cam_model('birdview')
P=scene.to_world(D,K,R,t)
np.save('snaps/bird_P2.npy',P)
X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(np.abs(X-0.009)<0.08)&(np.abs(Y-0.007)<0.08)&(Z>0.91)
print("mug pts",m.sum(),"zmax",Z[m].max())
for zt in [0.93,0.95,0.96,0.97,0.975,0.98,0.99,1.0]:
    mm=m&(Z>zt)
    if mm.sum(): print(zt,mm.sum(),"x",np.round([X[mm].min(),X[mm].max()],3),"y",np.round([Y[mm].min(),Y[mm].max()],3),"c",np.round([X[mm].mean(),Y[mm].mean()],3))
