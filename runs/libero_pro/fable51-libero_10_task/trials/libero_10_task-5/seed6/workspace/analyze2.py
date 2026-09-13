import numpy as np, cv2
P=np.load("birdview_cloud.npy"); img=cv2.imread("birdview.png")
z=P[:,:,2]
def rep(name, m):
    pts=P[m]; print(name, "n",m.sum(), "x[%.3f %.3f] y[%.3f %.3f] z pct 5/50/95/max %.3f %.3f %.3f %.3f"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),*np.percentile(pts[:,2],[5,50,95]),pts[:,2].max()))
    print("  centroid xy", pts[:,0].mean(), pts[:,1].mean())
# cup region
m=(z>0.895); sub=np.zeros_like(m); sub[248:280,305:350]=True; rep("cup", m&sub)
# cup body only (exclude handle): use color - cup body whitish/gray
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
# caddy: brown box; restrict to region x in [-0.6,-0.25], y in [-0.36,0.13], z in [0.9,1.05]
m2=(z>0.93)&(z<1.1)&(P[:,:,0]>-0.62)&(P[:,:,0]<-0.2)&(P[:,:,1]>-0.4)&(P[:,:,1]<0.15)
rep("caddy walls", m2)
# print z map of caddy region coarsely
ys,xs=np.where(m2); print("caddy px bbox", xs.min(),xs.max(),ys.min(),ys.max())
# per row in caddy region print z
for v in range(160,210,4):
    print(v, " ".join("%.2f"%z[v,u] for u in range(210,310,4)))
