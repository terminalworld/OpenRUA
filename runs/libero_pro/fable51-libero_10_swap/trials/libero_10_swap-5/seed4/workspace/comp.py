import numpy as np, cv2
P = np.load("birdview_xyz.npy"); z = P[...,2]
u0,v0,w,h = 205,150,115,70
sub = z[v0:v0+h, u0:u0+w]
floor = ((sub>0.885)&(sub<0.93)).astype(np.uint8)
# exclude table outside caddy: floor pixel must be inside caddy bbox: use connected components
n, lab, stats, cent = cv2.connectedComponentsWithStats(floor)
for i in range(1,n):
    if stats[i,4]<20: continue
    m = np.zeros_like(z,bool); m[v0:v0+h,u0:u0+w] = lab==i
    pts = P[m]
    print(f"comp {i} area {stats[i,4]} bbox {stats[i,:4]+np.array([u0,v0,0,0])} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]")
# wall top heights
walls = P[v0:v0+h,u0:u0+w][(sub>1.0)&(sub<1.2)]
print("wall top z", np.percentile(walls[:,2],[5,50,95]))
