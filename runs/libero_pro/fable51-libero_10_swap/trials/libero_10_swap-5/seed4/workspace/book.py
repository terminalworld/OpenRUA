import numpy as np, cv2
P = np.load("birdview_xyz.npy"); z = P[...,2]
m = (z>0.9)&(z<1.2)&(P[...,0]>-0.2)&(P[...,0]<0.0)&(P[...,1]>-0.1)&(P[...,1]<0.06)
pts = P[m]
print("n", len(pts), "z", pts[:,2].min(), pts[:,2].max())
top = pts[pts[:,2]>1.06]
print("top n", len(top), "z", top[:,2].min(), top[:,2].max())
xy = top[:,:2].astype(np.float32)
rect = cv2.minAreaRect(xy)
print("minAreaRect center", rect[0], "size", rect[1], "angle", rect[2])
# PCA
c = xy.mean(0); u,s,vt = np.linalg.svd(xy-c)
print("center", c, "axes", vt, "extent along axes", [(xy-c)@vt[i] for i in range(2)][0].ptp(), ((xy-c)@vt[1]).ptp())
# also full footprint
xy2 = pts[:,:2].astype(np.float32)
rect2 = cv2.minAreaRect(xy2); print("footprint rect", rect2)
