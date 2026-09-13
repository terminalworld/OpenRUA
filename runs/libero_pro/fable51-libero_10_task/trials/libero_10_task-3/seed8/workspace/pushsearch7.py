import numpy as np
from pushsearch import rot_axis, hand_pts_fine
from pushsearch5 import ok_pose, SH
from robot import rot_from_axes, TCP_OFF
best=[]
for tilt in np.arange(0,80,5):
    for az in np.arange(0,360,15):
        if tilt==0 and az>0: continue
        zh=np.array([np.sin(np.radians(tilt))*np.cos(np.radians(az)),np.sin(np.radians(tilt))*np.sin(np.radians(az)),-np.cos(np.radians(tilt))])
        ref=np.array([1,0,0]) if abs(zh[0])<0.9 else np.array([0,1,0]); y0=np.cross(zh,ref); y0/=np.linalg.norm(y0)
        for roll in np.arange(0,180,10):
            yh=rot_axis(zh,np.radians(roll))@y0; R=rot_from_axes(zh,yh)
            for fsign in (-1,0,1):
                finger=0.04 if fsign else 0.0
                for xc in (-0.02,0.0,0.02):
                    for zc in (0.935,0.945):
                        m=-1
                        for mm in (0.0,0.005,0.01,0.015,0.02,0.025):
                            good=True
                            for yw in (0.14,0.16,0.18):
                                tcp=np.array([xc,yw-0.012,zc])-0.0086*zh+fsign*0.05*yh
                                if np.linalg.norm(tcp-TCP_OFF*zh-SH)>0.80: good=False;break
                                if not ok_pose(tcp,R,mm,('drawer_walls','low_handle'),finger): good=False;break
                            if good: m=mm
                            else: break
                        if m>=0: best.append((m,tilt,az,roll,fsign,xc,zc))
best.sort(key=lambda t:-t[0])
print(len(best))
for b in best[:25]: print(np.round(b,3))
