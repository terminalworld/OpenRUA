import numpy as np, sys
P=np.load(sys.argv[1]); 
res=0.004
x0,x1,y0,y1=-0.50,-0.30,-0.40,0.10
W=int((x1-x0)/res); H=int((y1-y0)/res)
hm=np.full((H,W),np.nan)
Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
sel=(Q[:,0]>x0)&(Q[:,0]<x1)&(Q[:,1]>y0)&(Q[:,1]<y1)&(Q[:,2]<1.2)
Q=Q[sel]
ix=((Q[:,0]-x0)/res).astype(int); iy=((Q[:,1]-y0)/res).astype(int)
for a,b,z in zip(ix,iy,Q[:,2]):
    if np.isnan(hm[b,a]) or z>hm[b,a]: hm[b,a]=z
print("cols x from %.3f step %.3f; values mm above table 0.880"%(x0,res))
print("       ", "".join("%4d"%int(round((x0+i*res)*1000)) for i in range(W)))
for j in range(H):
    row=hm[j]
    s="".join("   ." if np.isnan(v) else ("%4d"%int(round((v-0.88)*1000))) for v in row)
    print("y=%+.3f"%(y0+j*res), s)
