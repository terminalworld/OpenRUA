import numpy as np, cv2, sys
cams=sys.argv[1].split(",")
res=0.005
x0,x1,y0,y1=-0.6,0.5,-0.5,0.5
nx,ny=int((x1-x0)/res),int((y1-y0)/res)
hm=np.full((ny,nx),np.nan)
for cam in cams:
    pw=np.load(f"{cam}_cloud.npy").reshape(-1,3)
    ok=np.isfinite(pw).all(1); pw=pw[ok]
    ix=((pw[:,0]-x0)/res).astype(int); iy=((pw[:,1]-y0)/res).astype(int)
    m=(ix>=0)&(ix<nx)&(iy>=0)&(iy<ny); ix,iy,z=ix[m],iy[m],pw[m,2]
    # max height per cell
    order=np.argsort(z); 
    cur=hm[iy[order],ix[order]]
    hm[iy[order],ix[order]]=np.where(np.isnan(cur),z[order],np.maximum(cur,z[order]))
np.save("hmap.npy",hm)
# table height estimate
tab=np.nanmedian(hm); print("table z median",tab)
vals=hm[np.isfinite(hm)]
print("percentiles",np.percentile(vals,[1,5,50,90,99,100]))
# visualize: height above table, color scaled 0..0.3m
rel=np.clip((hm-tab)/0.3,0,1); rel[np.isnan(hm)]=0
img=(rel*255).astype(np.uint8); img=cv2.applyColorMap(img,cv2.COLORMAP_JET)
img=cv2.resize(img,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST)
# grid lines every 0.1 m
for xv in np.arange(x0,x1+1e-6,0.1):
    px=int((xv-x0)/res*4); cv2.line(img,(px,0),(px,img.shape[0]),(255,255,255),1); cv2.putText(img,f"{xv:.1f}",(px+2,12),cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,255,255),1)
for yv in np.arange(y0,y1+1e-6,0.1):
    py=int((yv-y0)/res*4); cv2.line(img,(0,py),(img.shape[1],py),(255,255,255),1); cv2.putText(img,f"{yv:.1f}",(2,py-2),cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,255,255),1)
cv2.imwrite("hmap.png",img); print("hmap.png", img.shape)
