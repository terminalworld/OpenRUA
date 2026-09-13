from robot import *
from plan3 import obstacles
for tilt in (15,25):
    c,s=np.cos(np.radians(tilt)),np.sin(np.radians(tilt))
    R=rot_from_axes([0,c,-s],[1,0,0])
    tcp=np.array([0.10,0.20,0.965]); p=tcp-TCP_OFF*R[:,2]
    hp=hand_points(p,R,0.0)
    x,y,z=hp.T
    print(tilt,'cab',np.round(hp[(y>0.21)&(z<1.135)],3),'han',np.round(hp[(x>-0.055)&(x<0.045)&(y>0.175)&(y<0.215)&(z>0.995)&(z<1.105)],3))
    print(' ymax',y.max(),'zmin',z.min(),'zmax',z.max())
