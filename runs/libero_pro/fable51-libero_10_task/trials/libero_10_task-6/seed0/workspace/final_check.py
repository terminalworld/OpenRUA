import numpy as np
from scipy.optimize import least_squares
from scipy import ndimage
d=np.load("agentview_cloud.npz"); pc=d["pc"]; col=d["col"]; z=pc[...,2]
# table height
print("table z:", np.round(np.median(z[np.isfinite(z)&(np.abs(pc[...,0]-0.1)<0.05)&(np.abs(pc[...,1]+0.3)<0.05)]),4))
# plate (flat, z 0.44-0.46, light color) footprint
pl=np.isfinite(z)&(z>0.440)&(z<0.462)&(np.abs(pc[...,0]-0.126)<0.12)&(np.abs(pc[...,1]-0.016)<0.12)&(col[...,0]>120)
P=pc[pl]; print(f"plate: n={len(P)} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] -> center ({(P[:,0].min()+P[:,0].max())/2:.3f},{(P[:,1].min()+P[:,1].max())/2:.3f}) radius~{(P[:,1].max()-P[:,1].min())/2:.3f}")
plate_c=np.array([(P[:,0].min()+P[:,0].max())/2,(P[:,1].min()+P[:,1].max())/2]); plate_r=(P[:,1].max()-P[:,1].min())/2
# red mug: circle fits at several heights
reg=np.isfinite(z)&(np.abs(pc[...,0]-0.126)<0.12)&(np.abs(pc[...,1]-0.016)<0.10)&(z>0.47)&(z<0.65)
M=pc[reg]; print("mug top z", M[:,2].max().round(4), " mug bottom-ish z", M[:,2].min().round(4))
for zl,zh in [(0.48,0.52),(0.52,0.56),(0.56,0.60)]:
    s=(M[:,2]>zl)&(M[:,2]<zh); Q=M[s][:,:2]
    f=lambda c: np.hypot(Q[:,0]-c[0],Q[:,1]-c[1])-c[2]
    r=least_squares(f,[0.126,0.02,0.04])
    print(f"  mug z[{zl},{zh}] center=({r.x[0]:.3f},{r.x[1]:.3f}) r={r.x[2]:.3f} rms={np.sqrt(np.mean(r.fun**2)):.4f}  dist to plate center={np.hypot(*(r.x[:2]-plate_c)):.3f} (plate r {plate_r:.3f})")
# pudding
pd=np.isfinite(z)&(z>0.44)&(z<0.48)&(pc[...,0]>0.02)&(pc[...,0]<0.25)&(pc[...,1]>0.10)&(pc[...,1]<0.30)
B=pc[pd]; print(f"pudding: n={len(B)} x[{B[:,0].min():.3f},{B[:,0].max():.3f}] y[{B[:,1].min():.3f},{B[:,1].max():.3f}] top z={B[:,2].max():.3f} center=({(B[:,0].min()+B[:,0].max())/2:.3f},{(B[:,1].min()+B[:,1].max())/2:.3f})")
print(f"pudding y-min {B[:,1].min():.3f} vs plate y-max {P[:,1].max():.3f} -> gap {B[:,1].min()-P[:,1].max():.3f} m; pudding is at +y (agentview image-right) of plate")
