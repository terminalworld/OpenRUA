import numpy as np, cv2, sys
P=np.load(sys.argv[1]); im=cv2.imread(sys.argv[2]); out=sys.argv[3]
x0,x1,y0,y1=map(float,sys.argv[4:8]); res=0.001
W=int((x1-x0)/res); H=int((y1-y0)/res)
img=np.zeros((H,W,3),np.uint8); zb=np.full((H,W),-1.0)
X,Y,Z=P[...,0],P[...,1],P[...,2]
m=np.isfinite(Z)&(X>=x0)&(X<x1)&(Y>=y0)&(Y<y1)
us=np.clip(((X[m]-x0)/res).astype(int),0,W-1); vs=np.clip(((Y[m]-y0)/res).astype(int),0,H-1)
cols=im[m]; zs=Z[m]
order=np.argsort(zs)   # paint highest last
for u,v,c,z in zip(us[order],vs[order],cols[order],zs[order]):
    img[v,u]=c; zb[v,u]=z
# image row = y (increasing downward -> flip so +y is up), col = x
img=np.ascontiguousarray(img[::-1])
# grid every 2cm
for xv in np.arange(np.ceil(x0/0.02)*0.02,x1,0.02):
    u=int((xv-x0)/res); cv2.line(img,(u,0),(u,H-1),(0,0,255) if abs(xv%0.1)<1e-6 or abs(xv%0.1-0.1)<1e-6 else (0,0,120),1)
    cv2.putText(img,f'{xv:.2f}',(u+1,H-3),cv2.FONT_HERSHEY_PLAIN,0.5,(0,255,255),1)
for yv in np.arange(np.ceil(y0/0.02)*0.02,y1,0.02):
    v=H-1-int((yv-y0)/res); cv2.line(img,(0,v),(W-1,v),(255,0,0) if abs(yv%0.1)<1e-6 or abs(yv%0.1-0.1)<1e-6 else (120,0,0),1)
    cv2.putText(img,f'{yv:.2f}',(2,v-2),cv2.FONT_HERSHEY_PLAIN,0.5,(0,255,255),1)
img=cv2.resize(img,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST)
cv2.imwrite(out,img); print(out,img.shape)
