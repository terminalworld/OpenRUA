import numpy as np, cv2
B = np.load("snaps/birdview_xyz.npy").reshape(-1,3)
A = np.load("snaps/agentview_xyz.npy").reshape(-1,3)
S = np.load("snaps/sideview_xyz.npy").reshape(-1,3)
P = np.concatenate([B,A,S]); P = P[np.isfinite(P).all(1)]
res=0.005; x0,x1,y0,y1 = -0.45,0.35,-0.35,0.55
W=int((y1-y0)/res); H=int((x1-x0)/res)
hm = np.full((H,W), np.nan)
ix=((P[:,0]-x0)/res).astype(int); iy=((P[:,1]-y0)/res).astype(int)
m=(ix>=0)&(ix<H)&(iy>=0)&(iy<W)&(P[:,2]<1.3)
for a,b,z in zip(ix[m],iy[m],P[m,2]):
    if np.isnan(hm[a,b]) or z>hm[a,b]: hm[a,b]=z
np.save("snaps/hm.npy", hm)
img = np.nan_to_num((hm-0.88)/(1.2-0.88),nan=0).clip(0,1)
img = cv2.applyColorMap((img*255).astype(np.uint8), cv2.COLORMAP_JET)
img = cv2.resize(img, (W*2,H*2), interpolation=cv2.INTER_NEAREST)
# grid lines every 0.1 m
for xv in np.arange(-0.4,0.35,0.1):
    r=int((xv-x0)/res)*2; cv2.line(img,(0,r),(W*2,r),(255,255,255),1); cv2.putText(img,f"x={xv:.1f}",(2,r-2),cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,255,255),1)
for yv in np.arange(-0.3,0.55,0.1):
    c=int((yv-y0)/res)*2; cv2.line(img,(c,0),(c,H*2),(255,255,255),1); cv2.putText(img,f"y={yv:.1f}",(c+2,12),cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,255,255),1)
cv2.imwrite("snaps/heightmap.png", img)
print("saved; rows=x (down = +x), cols=y (right = +y)")
