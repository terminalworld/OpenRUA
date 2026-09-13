import numpy as np
from plan3 import obstacles
from robot import rot_from_axes, hand_points, TCP_OFF
def rot_axis(axis,ang):
    axis=np.asarray(axis,float); axis/=np.linalg.norm(axis)
    K=np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0]])
    return np.eye(3)+np.sin(ang)*K+(1-np.cos(ang))*K@K
def hand_pts_fine(tcp,R,finger=0.0):
    pos=tcp-TCP_OFF*R[:,2]
    pts=[]
    for xs in np.linspace(-0.032,0.032,3):
        for ys in np.linspace(-0.1,0.1,9):
            for zs in np.linspace(0,0.066,4):
                pts.append(pos+R@np.array([xs,ys,zs]))
    for ys in (-finger-0.01,finger+0.01):
        for zs in (0.066,0.09,0.112):
            for xs in (-0.01,0.01):
                pts.append(pos+R@np.array([xs,ys,zs]))
    return np.array(pts)
def bad(pts,skip=()):
    h=[o for o in obstacles(pts) if o not in skip]
    if (pts[:,2]<0.91).any(): h.append('table')
    return h
results=[]
for tilt in np.radians(np.arange(0,80,5)):
    for az in np.radians(np.arange(0,360,15)):
        zh=np.array([np.sin(tilt)*np.cos(az),np.sin(tilt)*np.sin(az),-np.cos(tilt)])
        if tilt==0 and az>0: continue
        # reference perpendicular
        ref=np.array([1,0,0]) if abs(zh[0])<0.9 else np.array([0,1,0])
        y0=np.cross(zh,ref); y0/=np.linalg.norm(y0)
        for roll in np.radians(np.arange(0,180,15)):
            yh=rot_axis(zh,roll)@y0
            R=rot_from_axes(zh,yh)
            for xc in (0.06,0.08,0.10,0.115):
                for zc in (0.94,0.955,0.97):
                    ok=True; margin=9
                    for ywall in (0.073,0.213):
                        tip=np.array([xc,ywall-0.012,zc])
                        tcp=tip-0.0086*zh
                        pts=hand_pts_fine(tcp,R)
                        h=bad(pts,skip=('drawer_walls',))
                        if ywall==0.213: h=[o for o in h if o!='cabinet' or True]
                        if h: ok=False; break
                    if ok:
                        results.append((np.degrees(tilt),np.degrees(az),np.degrees(roll),xc,zc))
print(len(results))
for r in results[:60]: print(np.round(r,1))
