import sys, numpy as np, cv2
x0,x1,y0,y1=map(float,sys.argv[1:5]); out=sys.argv[5]
C=np.load("robot0_eye_in_hand_cloud.npy").reshape(-1,3); C=C[np.isfinite(C).all(1)]
m=(C[:,2]>0.903)&(C[:,2]<1.10)&(C[:,0]>x0)&(C[:,0]<x1)&(C[:,1]>y0)&(C[:,1]<y1)
P=C[m]; print("n",len(P),"zmax",P[:,2].max().round(3) if len(P) else None)
nx,ny=int((x1-x0)*1000),int((y1-y0)*1000); H=np.zeros((nx,ny))
ix=((P[:,0]-x0)*1000).astype(int); iy=((P[:,1]-y0)*1000).astype(int)
for a,b,z in zip(ix,iy,P[:,2]):
    if 0<=a<nx and 0<=b<ny: H[a,b]=max(H[a,b],z)
img=np.zeros((nx,ny,3),np.uint8); v=np.clip((H-0.90)/0.10,0,1); img[...,1]=(v*255).astype(np.uint8); img[H>0,2]=80
img=cv2.resize(img,(ny*2,nx*2),interpolation=cv2.INTER_NEAREST)
for k in range(0,max(nx,ny),50):
    cv2.line(img,(k*2,0),(k*2,nx*2-1),(60,60,60),1); cv2.line(img,(0,k*2),(ny*2-1,k*2),(60,60,60),1)
    cv2.putText(img,f"y{y0+k/1000:.2f}",(k*2,12),cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,255,255),1)
    cv2.putText(img,f"x{x0+k/1000:.2f}",(0,k*2+12),cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,255,255),1)
cv2.imwrite(out,img)
