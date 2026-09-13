import numpy as np, cv2
d = np.load("birdview_cloud.npz"); Pw, color = d["pw"], d["color"]
z = Pw[...,2]
# table height near the plate ring
m = np.isfinite(z) & (Pw[...,0]>0.0)&(Pw[...,0]<0.4)&(np.abs(Pw[...,1])<0.3)
hs, es = np.histogram(z[m], bins=np.arange(0.38,0.50,0.005)); print("table region z hist:", {round(e,3):int(c) for e,c in zip(es,hs) if c>0})
# plate: ring; centroid of all plate pixels
m = np.isfinite(z) & (Pw[...,0]>0.05)&(Pw[...,0]<0.25)&(np.abs(Pw[...,1])<0.12)&(z>0.44)
pts = Pw[m]; print("plate centroid", pts[:,0].mean().round(4), pts[:,1].mean().round(4), "n", m.sum(), "radius est", ((pts[:,0].max()-pts[:,0].min())/2).round(3), ((pts[:,1].max()-pts[:,1].min())/2).round(3))
# red mug: top rim points (z>0.54) in birdview to get body center
m = np.isfinite(z) & (Pw[...,0]>-0.27)&(Pw[...,0]<-0.13)&(Pw[...,1]>-0.07)&(Pw[...,1]<0.11)&(z>0.54)&(z<0.60)
pts = Pw[m]; print("redmug rim n", m.sum(), "x", pts[:,0].min().round(3), pts[:,0].max().round(3), "y", pts[:,1].min().round(3), pts[:,1].max().round(3), "mean", pts[:,0].mean().round(4), pts[:,1].mean().round(4))
ys, xs = np.where(m); vis = color.copy(); vis[m]=(0,255,0)
# white mug rim
m2 = np.isfinite(z) & (Pw[...,0]>-0.16)&(Pw[...,0]<-0.04)&(Pw[...,1]>-0.25)&(Pw[...,1]<-0.10)&(z>0.52)
pts = Pw[m2]; print("whitemug rim n", m2.sum(), "x", pts[:,0].min().round(3), pts[:,0].max().round(3), "y", pts[:,1].min().round(3), pts[:,1].max().round(3), "mean", pts[:,0].mean().round(4), pts[:,1].mean().round(4))
vis[m2]=(255,0,0)
# pudding top face (z 0.46-0.47)
m3 = np.isfinite(z) & (Pw[...,0]>-0.10)&(Pw[...,0]<0.02)&(Pw[...,1]>0.02)&(Pw[...,1]<0.14)&(z>0.455)&(z<0.48)
pts = Pw[m3]; print("pudding top n", m3.sum(), "x", pts[:,0].min().round(3), pts[:,0].max().round(3), "y", pts[:,1].min().round(3), pts[:,1].max().round(3), "mean", pts[:,0].mean().round(4), pts[:,1].mean().round(4), "z", pts[:,2].mean().round(4))
vis[m3]=(0,0,255)
# PCA for pudding orientation
c = pts[:,:2]-pts[:,:2].mean(0); w,v = np.linalg.eigh(c.T@c); print("pudding long axis (x,y):", v[:,1].round(3), "yaw deg", np.degrees(np.arctan2(v[1,1], v[0,1])).round(1))
crop = vis[200:340, 240:400]; cv2.imwrite("bird_crop.png", cv2.resize(cv2.cvtColor(crop, cv2.COLOR_RGB2BGR), None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
