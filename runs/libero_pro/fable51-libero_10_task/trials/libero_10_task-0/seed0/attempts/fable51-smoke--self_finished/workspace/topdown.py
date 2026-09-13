import numpy as np, cv2
Pw=np.load('agentview_xyz.npy'); img=cv2.imread('agentview.png')
res=0.005
x0,x1,y0,y1=-0.45,0.35,-0.45,0.45
W=int((x1-x0)/res); H=int((y1-y0)/res)
hm=np.full((H,W),np.nan); cm=np.zeros((H,W,3),np.uint8)
z=Pw[...,2]
sel=np.isfinite(z)&(z>0.43)&(z<0.8)&(Pw[...,0]>x0)&(Pw[...,0]<x1)&(Pw[...,1]>y0)&(Pw[...,1]<y1)
xs=((Pw[...,0][sel]-x0)/res).astype(int); ys=((Pw[...,1][sel]-y0)/res).astype(int)
zs=z[sel]; cs=img[sel]
order=np.argsort(zs)
for i in order:
    hm[ys[i],xs[i]]=zs[i]; cm[ys[i],xs[i]]=cs[i]
# draw: rows = y, cols = x; flip so +x down? keep: col=x, row=y
big=cv2.resize(cm,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST)
# grid lines every 10cm
for gx in np.arange(x0,x1+1e-9,0.1):
    c=int((gx-x0)/res)*4; cv2.line(big,(c,0),(c,big.shape[0]-1),(80,80,80),1); cv2.putText(big,f"{gx:.1f}",(c+2,12),cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,255,255),1)
for gy in np.arange(y0,y1+1e-9,0.1):
    r=int((gy-y0)/res)*4; cv2.line(big,(0,r),(big.shape[1]-1,r),(80,80,80),1); cv2.putText(big,f"{gy:.1f}",(2,r-2),cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,255,255),1)
cv2.imwrite('topdown.png',big)
np.save('hm.npy',hm)
