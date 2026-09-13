import numpy as np, sys
from rlib import *
import rclpy
r=Robot()
d=np.load("snaps/robot0_eye_in_hand_depth.npy")
fx=312.77408948188935; cx=320; cy=240
pos,quat=r.tf("world","robot0_eye_in_hand_optical_frame")
from scipy.spatial.transform import Rotation as Ro
R=Ro.from_quat(quat).as_matrix()
v,u=np.mgrid[0:480,0:640]
z=d; X=(u-cx)*z/fx; Y=(v-cy)*z/fx
P=np.stack([X,Y,z],-1).reshape(-1,3)@R.T+pos
ok=np.isfinite(z.reshape(-1))&(z.reshape(-1)>0.05)
P=P[ok]
print("cam",np.round(pos,3))
face=P[(P[:,1]>0.235)&(P[:,1]<0.27)]
print("front face pts",len(face))
# histogram in z, x
for zlo in np.arange(0.88,1.20,0.02):
    s=face[(face[:,2]>=zlo)&(face[:,2]<zlo+0.02)]
    if len(s): print(f"face z[{zlo:.2f},{zlo+0.02:.2f}) n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}]")
inside=P[(P[:,1]>0.29)&(P[:,1]<0.46)]
print("inside pts",len(inside))
for zlo in np.arange(0.88,1.20,0.02):
    s=inside[(inside[:,2]>=zlo)&(inside[:,2]<zlo+0.02)]
    if len(s): print(f"in z[{zlo:.2f},{zlo+0.02:.2f}) n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
for xlo in np.arange(-0.25,0.15,0.02):
    s=inside[(inside[:,0]>=xlo)&(inside[:,0]<xlo+0.02)]
    if len(s): print(f"in x[{xlo:.2f},{xlo+0.02:.2f}) n={len(s)} z[{s[:,2].min():.3f},{s[:,2].max():.3f}]")
