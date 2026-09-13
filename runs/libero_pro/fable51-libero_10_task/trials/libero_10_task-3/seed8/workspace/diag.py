import numpy as np
from pushsearch import rot_axis, hand_pts_fine
from plan3 import obstacles
from robot import rot_from_axes, TCP_OFF, BASE_IN_WORLD
SH=np.array(BASE_IN_WORLD)+np.array([0,0,0.333])
def diag(tilt,az,roll,fsign,xc,zc,m=0.005):
    tilt,az,roll=np.radians([tilt,az,roll])
    zh=np.array([np.sin(tilt)*np.cos(az),np.sin(tilt)*np.sin(az),-np.cos(tilt)])
    ref=np.array([1,0,0]) if abs(zh[0])<0.9 else np.array([0,1,0])
    y0=np.cross(zh,ref); y0/=np.linalg.norm(y0)
    yh=rot_axis(zh,roll)@y0; R=rot_from_axes(zh,yh)
    finger=0.04 if fsign else 0.0
    for yw in (0.14,0.18,0.213):
        tcp=np.array([xc,yw-0.012,zc])-0.0086*zh+fsign*0.05*yh
        origin=tcp-TCP_OFF*zh
        pts=hand_pts_fine(tcp,R,finger)
        hits=set()
        for i in range(3):
            for s in (-1,0,1):
                p=pts+s*m*np.eye(3)[i]
                for h in obstacles(p): hits.add(h)
                if (p[:,2]<0.91).any(): hits.add('table')
        print(f'yw {yw} tcp {np.round(tcp,3)} reach {np.linalg.norm(origin-SH):.3f} hits {sorted(hits)}  ymax {pts[:,1].max():.3f} zmin {pts[:,2].min():.3f}')
    print('R=\n',np.round(R,3))
for c in [(40,135,80,1,0.10,0.975),(50,135,90,1,0.08,0.975),(40,90,0,1,0.115,0.975),(40,90,0,0,0.115,0.975)]:
    print(c); diag(*c)
print('---- detail')
def detail(tilt,az,roll,fsign,xc,zc,yw=0.213,m=0.0):
    tilt,az,roll=np.radians([tilt,az,roll])
    zh=np.array([np.sin(tilt)*np.cos(az),np.sin(tilt)*np.sin(az),-np.cos(tilt)])
    ref=np.array([1,0,0]) if abs(zh[0])<0.9 else np.array([0,1,0])
    y0=np.cross(zh,ref); y0/=np.linalg.norm(y0)
    yh=rot_axis(zh,roll)@y0; R=rot_from_axes(zh,yh)
    finger=0.04 if fsign else 0.0
    tcp=np.array([xc,yw-0.012,zc])-0.0086*zh+fsign*0.05*yh
    pts=hand_pts_fine(tcp,R,finger)
    x,y,z=pts.T
    cab=(x>-0.12-m)&(x<0.13+m)&(y>0.21-m)&(z<1.135+m)
    han=(x>-0.055-m)&(x<0.045+m)&(y>0.175-m)&(y<0.215+m)&(z>0.995-m)&(z<1.105+m)
    print('tcp',np.round(tcp,3)); print('cabinet pts',np.round(pts[cab],3)); print('handle pts',np.round(pts[han],3))
    print('R',np.round(R,3))
detail(50,135,90,1,0.08,0.975,m=0.005)
print('=====')
for c in [(60,135,100,1,0.06,0.975),(70,135,110,1,0.06,0.975),(40,135,80,1,0.10,0.975)]:
    print(c); detail(*c,m=0.005)
