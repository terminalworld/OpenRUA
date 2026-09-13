import numpy as np, yaml, subprocess, sys
CAMS = {
 "birdview": ((-0.2,0,3.0),(0.7071,0.7071,0,0)),
 "sideview": ((-0.0565,1.2761,1.4880),(0.0099,0.8064,-0.5912,-0.0069)),
 "frontview": ((1.0,0,1.48),(0.5608,0.5608,-0.4306,-0.4306)),
 "agentview": ((0.6586,0,1.6104),(0.6380,0.6380,-0.3048,-0.3048)),
}
def qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def cloud(cam, npy, fx, cx=320, cy=240):
    d = np.load(npy); H,W = d.shape
    vv,uu = np.mgrid[0:H,0:W]
    X = (uu-cx)*d/fx; Y=(vv-cy)*d/fx
    t,q = CAMS[cam]; R = qR(*q)
    P = np.stack([X,Y,d],-1) @ R.T + np.array(t)
    return P
if __name__ == "__main__":
    import re
    cam = sys.argv[1]; npy = sys.argv[2]
    info = subprocess.run(["ros2","topic","echo",f"/{cam}/color/camera_info","--once"],capture_output=True,text=True,timeout=30).stdout
    k = [float(v) for v in re.findall(r"^- ([\d.e+-]+)", info.split("k:")[1].split("r:")[0], re.M)]
    fx = k[0]; print("fx",fx)
    P = cloud(cam,npy,fx)
    np.save(f"{cam}_cloud.npy", P)
    flat = P.reshape(-1,3)
    # table-plane check
    tab = flat[(np.abs(flat[:,2]-0.90)<0.005)]
    print("table pts", len(tab), "z mean", tab[:,2].mean())
    def region(xr, yr, name):
        m = (P[...,0]>xr[0])&(P[...,0]<xr[1])&(P[...,1]>yr[0])&(P[...,1]<yr[1])&(P[...,2]>0.905)&(P[...,2]<1.2)
        pts = P[m]; print(name, len(pts))
        for lo in np.arange(0.90, 1.07, 0.01):
            s = pts[(pts[:,2]>=lo)&(pts[:,2]<lo+0.01)]
            if len(s): print(f"  z[{lo:.2f},{lo+0.01:.2f}) n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}")
    region((-0.26,-0.15),(-0.30,-0.12),"potA")
    region((-0.13,-0.01),(0.13,0.31),"potB")
    region((0.08,0.30),(-0.08,0.14),"stove")
