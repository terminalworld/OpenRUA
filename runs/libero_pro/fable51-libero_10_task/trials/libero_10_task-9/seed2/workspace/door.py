import sys, arm, numpy as np
H=np.array([-0.19,0.247]); S=0.17; OFF=0.0145; Z=1.075
def d(th): t=np.radians(th); return np.array([np.cos(t),-np.sin(t)])
def n(th): t=np.radians(th); return np.array([-np.sin(t),-np.cos(t)])
def pose(th, extra=0.0):
    p=H+S*d(th)+(OFF+extra)*n(th)
    R=arm.R_from_axes([0,0,-1],[n(th)[0],n(th)[1],0])
    return np.array([p[0],p[1],Z]),R
a=arm.Arm()
thetas=[float(x) for x in sys.argv[1:]]
first=True
for th in thetas:
    if first:
        p,R=pose(th,0.04); a.move_tcp(p,R,seconds=4); first=False
    p,R=pose(th); a.move_tcp(p,R,seconds=3)
    print(f'theta {th}: tcp {np.round(a.tcp()[0],4)} w {np.round(a.wrench(),2)}')
