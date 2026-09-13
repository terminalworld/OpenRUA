import numpy as np
from pushsearch import rot_axis, hand_pts_fine, bad
from robot import rot_from_axes
def ok_pose(tcp,R,m):
    pts=hand_pts_fine(tcp,R)
    shifts=[np.zeros(3)]+[s*m*np.eye(3)[i] for i in range(3) for s in (-1,1)]
    for s in shifts:
        if bad(pts+s,skip=('drawer_walls',)): return False
    return True
best=[]
for tilt in np.radians(np.arange(30,75,2.5)):
    for az in np.radians(np.arange(100,160,5)):
        zh=np.array([np.sin(tilt)*np.cos(az),np.sin(tilt)*np.sin(az),-np.cos(tilt)])
        ref=np.array([1,0,0]); y0=np.cross(zh,ref); y0/=np.linalg.norm(y0)
        for roll in np.radians(np.arange(80,150,5)):
            yh=rot_axis(zh,roll)@y0; R=rot_from_axes(zh,yh)
            for xc in (0.09,0.10,0.11):
                for zc in (0.955,0.965,0.975):
                    m=0
                    for mm in (0.004,0.008,0.012,0.016):
                        good=True
                        for ywall in (0.073,0.14,0.213):
                            tip=np.array([xc,ywall-0.012,zc]); tcp=tip-0.0086*zh
                            if not ok_pose(tcp,R,mm): good=False;break
                        if good: m=mm
                        else: break
                    if m>0: best.append((m,np.degrees(tilt),np.degrees(az),np.degrees(roll),xc,zc))
best.sort(key=lambda t:-t[0])
print(len(best))
for b in best[:30]: print(np.round(b,3))
