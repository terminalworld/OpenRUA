import numpy as np
fx=fy=579.4112549695428; cx=320; cy=240
def R_from_q(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def cloud(npy, t, q):
    d = np.load(npy); H,W=d.shape; vs,us=np.mgrid[0:H,0:W]
    P = np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d],-1) @ R_from_q(*q).T + np.array(t)
    return P.reshape(-1,3)
cams = {"side":cloud("snaps/sideview_depth.npy", (-0.0565,1.2761,1.4880),(0.0099,0.8064,-0.5912,-0.0069)),
        "front":cloud("snaps/frontview_depth.npy", (1.0,0.0,1.48),(0.5608,0.5608,-0.4306,-0.4306))}
table=0.899
for name,(x0,x1,y0,y1) in {"potA":(-0.26,-0.15,-0.30,-0.13),"potB":(-0.13,-0.02,0.13,0.29)}.items():
  for cam,pts in cams.items():
    m = (pts[:,0]>x0)&(pts[:,0]<x1)&(pts[:,1]>y0)&(pts[:,1]<y1)&(pts[:,2]>table+0.005)
    p = pts[m]
    print(name, cam, len(p))
    for z0 in np.arange(0.0,0.16,0.005):
        s = p[(p[:,2]-table>=z0)&(p[:,2]-table<z0+0.005)]
        if len(s)<3: continue
        print(f"  h {z0*100:5.1f}cm n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] w={100*(s[:,0].max()-s[:,0].min()):.1f}  y[{s[:,1].min():.3f},{s[:,1].max():.3f}] w={100*(s[:,1].max()-s[:,1].min()):.1f}")
