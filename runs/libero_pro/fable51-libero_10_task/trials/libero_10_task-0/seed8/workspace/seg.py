import numpy as np, cv2
pc = np.load("/workspace/bird_pc3.npy")
img = cv2.imread("/workspace/bird3.png")
z = pc[...,2]
# table plane height: mode of z in the table region
tbl = z[(np.abs(pc[...,0])<0.35)&(np.abs(pc[...,1])<0.5)&np.isfinite(z)]
h,_ = np.histogram(tbl, bins=np.arange(0.3,0.7,0.002)); table_z = 0.3+0.002*(np.argmax(h)+0.5)
print("table z ~", round(table_z,4))
mask = (z > table_z+0.012) & (z < table_z+0.35) & np.isfinite(z) & (np.abs(pc[...,1])<0.5) & (pc[...,0]>-0.4) & (pc[...,0]<0.35)
n, lab = cv2.connectedComponents(mask.astype(np.uint8))
for i in range(1,n):
    m = lab==i
    if m.sum()<15: continue
    P = pc[m]
    ys,xs = np.nonzero(m)
    print(f"blob {i}: px({xs.mean():.0f},{ys.mean():.0f}) n={m.sum()} "
          f"center=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) x[{P[:,0].min():.3f},{P[:,0].max():.3f}] "
          f"y[{P[:,1].min():.3f},{P[:,1].max():.3f}] ztop={P[:,2].max():.3f} zmed={np.median(P[:,2]):.3f}")
cv2.imwrite("/workspace/bird_mask.png", (mask*255).astype(np.uint8))
print("--- PCA")
for i in (4,9):
    m = lab==i; P = pc[m][:,:2]; c = P.mean(0)
    w,v = np.linalg.eigh(np.cov((P-c).T)); major = v[:,1]
    ang = np.degrees(np.arctan2(major[1], major[0]))
    print(f"blob {i}: major axis angle from +x = {ang:.1f} deg; extents along major/minor: "
          f"{np.ptp((P-c)@v[:,1]):.3f} {np.ptp((P-c)@v[:,0]):.3f}")
