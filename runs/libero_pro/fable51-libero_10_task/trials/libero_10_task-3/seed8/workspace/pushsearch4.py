import numpy as np
from pushsearch import rot_axis, hand_pts_fine
from pushsearch3 import ok_pose
from robot import rot_from_axes
phases={'A':(0.073,0.11,0.15),'B':(0.14,0.18,0.213)}
for name,yw in phases.items():
    best=[]
    for tilt in np.radians(np.arange(0,80,5)):
        for az in np.radians(np.arange(0,360,15)):
            if tilt==0 and az>0: continue
            zh=np.array([np.sin(tilt)*np.cos(az),np.sin(tilt)*np.sin(az),-np.cos(tilt)])
            ref=np.array([1,0,0]) if abs(zh[0])<0.9 else np.array([0,1,0])
            y0=np.cross(zh,ref); y0/=np.linalg.norm(y0)
            for roll in np.radians(np.arange(0,180,10)):
                yh=rot_axis(zh,roll)@y0; R=rot_from_axes(zh,yh)
                for xc in (0.06,0.08,0.10,0.115):
                    for zc in (0.945,0.96,0.975):
                        m=-1
                        for mm in (0.0,0.005,0.01,0.015,0.02,0.025,0.03):
                            if all(ok_pose(np.array([xc,y-0.012,zc])-0.0086*zh,R,mm,('drawer_walls',)) for y in yw): m=mm
                            else: break
                        if m>=0: best.append((m,np.degrees(tilt),np.degrees(az),np.degrees(roll),xc,zc))
    best.sort(key=lambda t:-t[0])
    print(name,len(best))
    for b in best[:12]: print(np.round(b,3))
