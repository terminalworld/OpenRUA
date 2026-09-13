import numpy as np, cv2
P = np.load("birdview_xyz.npy"); z = P[...,2]
c = cv2.imread("birdview.png")
# table height
tab = z[(z>0.8)&(z<0.9)]
print("table z median", np.median(tab))
mask = ((z>0.895)&(z<1.2)).astype(np.uint8)
# exclude robot region: robot base at world x=-0.75; keep world x > -0.6
mask &= (P[...,0] > -0.62).astype(np.uint8)
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4] < 30: continue
    m = lab==i
    pts = P[m]
    print(f"comp {i}: px area {stats[i,4]} bbox(u,v,w,h)={stats[i,:4]} world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]")
