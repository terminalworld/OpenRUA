import scene, numpy as np, cv2
D,C,K,R,t=scene.cam_model("birdview"); cv2.imwrite("snaps/bird_s.png",C)
P=scene.to_world(D,K,R,t); pts=P[np.isfinite(P[...,2])]
# anything left where the open door was?
s=pts[(pts[:,0]>-0.36)&(pts[:,0]<-0.26)&(pts[:,1]>-0.66)&(pts[:,1]<-0.40)&(pts[:,2]>0.915)&(pts[:,2]<1.12)]
print("old door region pts (z<1.12)",len(s))
# door closed: front region
for xlo in np.arange(-0.30,0.08,0.05):
    f=pts[(pts[:,0]>=xlo)&(pts[:,0]<xlo+0.05)&(pts[:,1]>-0.45)&(pts[:,1]<-0.30)&(pts[:,2]>1.05)&(pts[:,2]<1.12)]
    if len(f): print(f" x{xlo:.2f}: n={len(f)} ymin={f[:,1].min():.3f} y1%={np.percentile(f[:,1],1):.3f} ztop={np.percentile(f[:,2],95):.3f}")
