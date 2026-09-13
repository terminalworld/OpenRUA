import numpy as np
from pushsearch import rot_axis, hand_pts_fine
from plan3 import obstacles
from robot import rot_from_axes, TCP_OFF, BASE_IN_WORLD
def bad(pts,skip):
    h=[o for o in obstacles(pts) if o not in skip]
    if (pts[:,2]<0.91).any(): h.append('table')
    return h
def ok_pose(tcp,R,m,skip,finger):
    pts=hand_pts_fine(tcp,R,finger)
    shifts=[np.zeros(3)]+[s*m*np.eye(3)[i] for i in range(3) for s in (-1,1)]
    return not any(bad(pts+s,skip) for s in shifts)
SH=np.array(BASE_IN_WORLD)+np.array([0,0,0.333])
best=[]
for tilt in np.radians(np.arange(0,80,5)):
    for az in np.radians(np.arange(0,360,15)):
        if tilt==0 and az>0: continue
        zh=np.array([np.sin(tilt)*np.cos(az),np.sin(tilt)*np.sin(az),-np.cos(tilt)])
        ref=np.array([1,0,0]) if abs(zh[0])<0.9 else np.array([0,1,0])
        y0=np.cross(zh,ref); y0/=np.linalg.norm(y0)
        for roll in np.radians(np.arange(0,180,10)):
            yh=rot_axis(zh,roll)@y0; R=rot_from_axes(zh,yh)
            for fsign in (-1,0,1):
                finger=0.04 if fsign else 0.0
                for xc in (0.06,0.08,0.10,0.115):
                    for zc in (0.945,0.96,0.975):
                        m=-1
                        for mm in (0.0,0.005,0.01,0.015,0.02,0.025):
                            good=True
                            for yw in (0.14,0.18,0.213):
                                tcp=np.array([xc,yw-0.012,zc])-0.0086*zh+fsign*0.05*yh
                                origin=tcp-TCP_OFF*zh
                                if np.linalg.norm(origin-SH)>0.80: good=False;break
                                if not ok_pose(tcp,R,mm,('drawer_walls',),finger): good=False;break
                            if good: m=mm
                            else: break
                        if m>=0: best.append((m,np.degrees(tilt),np.degrees(az),np.degrees(roll),fsign,xc,zc))
best.sort(key=lambda t:-t[0])
print(len(best))
for b in best[:25]: print(np.round(b,3))
