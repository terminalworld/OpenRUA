import scene, numpy as np, cv2
D,C,K,R,t=scene.cam_model("robot0_eye_in_hand")
cv2.imwrite("snaps/eih_r.png",C)
P=scene.to_world(D,K,R,t)
hsv=cv2.cvtColor(C,cv2.COLOR_BGR2HSV)
ok=np.isfinite(P[...,2])&(D>0.12)
yel=(hsv[...,0]>15)&(hsv[...,0]<40)&(hsv[...,1]>60)&(hsv[...,2]>35)&ok
pts=P[yel]
print("yellow n",yel.sum(),"x",np.round([pts[:,0].min(),pts[:,0].max()],3),"y",np.round([pts[:,1].min(),pts[:,1].max()],3),"z",np.round([pts[:,2].min(),pts[:,2].max()],3))
for lo,hi in [(0.94,0.97),(0.97,1.0),(1.0,1.03),(1.03,1.06)]:
    s=pts[(pts[:,2]>=lo)&(pts[:,2]<hi)]
    if len(s): print(f" z{lo}-{hi} n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] ymin1%={np.percentile(s[:,1],1):.3f}")
# everything (any colour) in the cavity box, beyond the face plane inward
allp=P[ok]
cav=allp[(allp[:,0]>-0.22)&(allp[:,0]<-0.05)&(allp[:,2]>0.95)&(allp[:,2]<1.08)&(allp[:,1]>-0.42)&(allp[:,1]<-0.17)]
print("cavity pts",len(cav),"y range",np.round([cav[:,1].min(),cav[:,1].max()],3),"y1%",np.round(np.percentile(cav[:,1],1),3))
# points that are in the opening x-range and z-range but y in [-0.40,-0.355] (would protrude)
pro=cav[cav[:,1]<-0.355]
print("protruding candidates",len(pro))
if len(pro): print(" x",np.round([pro[:,0].min(),pro[:,0].max()],3),"y",np.round([pro[:,1].min(),pro[:,1].max()],3),"z",np.round([pro[:,2].min(),pro[:,2].max()],3))
