import numpy as np
from pushsearch import rot_axis, hand_pts_fine
from plan3 import obstacles
from robot import rot_from_axes
def bad(pts,skip):
    h=[o for o in obstacles(pts) if o not in skip]
    if (pts[:,2]<0.91).any(): h.append('table')
    return h
def ok_pose(tcp,R,m,skip):
    pts=hand_pts_fine(tcp,R)
    shifts=[np.zeros(3)]+[s*m*np.eye(3)[i] for i in range(3) for s in (-1,1)]
    return not any(bad(pts+s,skip) for s in shifts)
modes={'wall':dict(xs=(0.06,0.08,0.10,0.115),zs=(0.945,0.96,0.975),ywalls=(0.073,0.14,0.213),skip=('drawer_walls',)),
       'handle':dict(xs=(-0.02,0.0,0.02),zs=(0.93,0.94,0.95),ywalls=(0.04,0.11,0.18),skip=('drawer_walls','low_handle'))}
for name,md in modes.items():
    best=[]
    for tilt in np.radians(np.arange(0,80,5)):
        for az in np.radians(np.arange(0,360,15)):
            if tilt==0 and az>0: continue
            zh=np.array([np.sin(tilt)*np.cos(az),np.sin(tilt)*np.sin(az),-np.cos(tilt)])
            ref=np.array([1,0,0]) if abs(zh[0])<0.9 else np.array([0,1,0])
            y0=np.cross(zh,ref); y0/=np.linalg.norm(y0)
            for roll in np.radians(np.arange(0,180,10)):
                yh=rot_axis(zh,roll)@y0; R=rot_from_axes(zh,yh)
                for xc in md['xs']:
                    for zc in md['zs']:
                        m=-1
                        for mm in (0.0,0.004,0.008,0.012,0.016):
                            if all(ok_pose(np.array([xc,yw-0.012,zc])-0.0086*zh,R,mm,md['skip']) for yw in md['ywalls']): m=mm
                            else: break
                        if m>=0: best.append((m,np.degrees(tilt),np.degrees(az),np.degrees(roll),xc,zc))
    best.sort(key=lambda t:-t[0])
    print(name,len(best))
    for b in best[:15]: print(np.round(b,3))
