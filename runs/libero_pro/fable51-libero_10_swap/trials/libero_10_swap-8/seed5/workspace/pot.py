import numpy as np
d = np.load("birdview.npy")
fx=fy=579.4112549695428; cx=320; cy=240
t = np.array([-0.2,0,3.0])
H,W = d.shape
vv,uu = np.mgrid[0:H,0:W]
X = (uu-cx)*d/fx; Y=(vv-cy)*d/fy
x,y,z,w = 0.7071,0.7071,0,0
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
P = np.stack([X,Y,d],-1) @ R.T + t
def region(xr, yr, name):
    m = (P[...,0]>xr[0])&(P[...,0]<xr[1])&(P[...,1]>yr[0])&(P[...,1]<yr[1])
    pts = P[m]
    print(name)
    for lo in np.arange(0.90, 1.07, 0.01):
        s = pts[(pts[:,2]>=lo)&(pts[:,2]<lo+0.01)]
        if len(s): print(f"  z[{lo:.2f},{lo+0.01:.2f}) n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
    top = pts[pts[:,2]>1.0]
    print("  body(z>1.0) mean", top[:,0].mean().round(4), top[:,1].mean().round(4))
region((-0.26,-0.15),(-0.30,-0.12),"potA")
region((-0.13,-0.01),(0.13,0.31),"potB")
region((0.08,0.30),(-0.08,0.14),"stove")
