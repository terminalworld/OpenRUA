import numpy as np
from rlib import *
from scipy.spatial.transform import Rotation as Ro
r=Robot()
d=np.load("snaps/robot0_eye_in_hand_depth.npy")
fx=312.77408948188935; cx=320; cy=240
pos,quat=r.tf("world","robot0_eye_in_hand_optical_frame")
R=Ro.from_quat(quat).as_matrix()
v,u=np.mgrid[0:480,0:640]
z=d; X=(u-cx)*z/fx; Y=(v-cy)*z/fx
P=np.stack([X,Y,z],-1).reshape(-1,3)@R.T+pos
ok=np.isfinite(z.reshape(-1))&(z.reshape(-1)>0.05); P=P[ok]
sel=P[(P[:,0]>-0.13)&(P[:,0]<-0.10)]
for ylo in np.arange(0.20,0.46,0.01):
    s=sel[(sel[:,1]>=ylo)&(sel[:,1]<ylo+0.01)]
    if len(s): print(f"y[{ylo:.2f}) n={len(s)} z[{s[:,2].min():.3f},{s[:,2].max():.3f}]")
