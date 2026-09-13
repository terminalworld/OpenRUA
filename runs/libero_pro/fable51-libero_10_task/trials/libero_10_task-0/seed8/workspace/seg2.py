import numpy as np, cv2
pc0 = np.load("/workspace/bird_pc.npy"); pc = np.load("/workspace/bird_pc3.npy")
z = pc[...,2]
# lying can blob
m = (z>0.437)&(z<0.56)&(pc[...,0]>-0.14)&(pc[...,0]<-0.02)&(pc[...,1]>0.03)&(pc[...,1]<0.15)
P = pc[m]; c = P[:,:2].mean(0)
w,v = np.linalg.eigh(np.cov((P[:,:2]-c).T)); major=v[:,1]
ang = np.degrees(np.arctan2(major[1],major[0]))
print(f"can: n={m.sum()} center={np.round(c,4)} major angle={ang:.1f} extents major={np.ptp((P[:,:2]-c)@v[:,1]):.3f} minor={np.ptp((P[:,:2]-c)@v[:,0]):.3f} ztop={P[:,2].max():.3f}")
# height profile along minor axis (should be a semicircle for a cylinder)
proj = (P[:,:2]-c)@v[:,0]
for lo in np.arange(-0.04,0.04,0.01):
    s=(proj>=lo)&(proj<lo+0.01)
    if s.any(): print(f"  minor {lo:+.2f}..{lo+0.01:+.2f}: zmax={P[s,2].max():.3f}")
# ridge top position (highest points) -> axis line
top = P[P[:,2]>P[:,2].max()-0.01]
print("ridge center", np.round(top[:,:2].mean(0),4), "n", len(top))
# basket interior diff
bi = (pc[...,0]>-0.06)&(pc[...,0]<0.045)&(pc[...,1]>0.19)&(pc[...,1]<0.32)
d = pc[...,2]-pc0[...,2]
print("basket interior: before zmed", np.round(np.median(pc0[bi][:,2]),3), "after zmed", np.round(np.median(pc[bi][:,2]),3),
      "max after", np.round(pc[bi][:,2].max(),3), "pixels raised >1.5cm:", int((d[bi]>0.015).sum()))
raised = bi & (d>0.015)
if raised.any():
    R = pc[raised]; print("raised region center", np.round(R[:,:2].mean(0),3), "z range", np.round(R[:,2].min(),3), np.round(R[:,2].max(),3),
                          "x", np.round(R[:,0].min(),3), np.round(R[:,0].max(),3), "y", np.round(R[:,1].min(),3), np.round(R[:,1].max(),3))
