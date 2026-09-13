import sys, numpy as np
P=np.load("birdview_world.npy"); v=np.isfinite(P[...,2])
x0,x1,y0,y1=[float(a) for a in sys.argv[1:5]]; zcut=float(sys.argv[5]) if len(sys.argv)>5 else 0.975
axis=sys.argv[6] if len(sys.argv)>6 else "y"   # profile along this axis
sel=v&(P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)&(P[...,2]>0.935)
pts=P[sel]; i=1 if axis=="y" else 0; j=1-i
lo,hi=(y0,y1) if axis=="y" else (x0,x1)
print(f"  {axis}    crest  {'x' if j==0 else 'y'}@crest | body extent (z>{zcut})           | all extent | n")
for s in np.arange(lo,hi,0.005):
    m=(pts[:,i]>=s)&(pts[:,i]<s+0.005)
    if not m.any(): continue
    q=pts[m]; k=q[:,2].argmax(); b=q[q[:,2]>zcut]
    bx=f"[{b[:,j].min():.3f},{b[:,j].max():.3f}] w={100*(b[:,j].max()-b[:,j].min()):4.1f} c={0.5*(b[:,j].min()+b[:,j].max()):.4f}" if len(b) else "-"
    print(f"{s:+.3f}  {q[k,2]:.4f}  {q[k,j]:.3f}  | {bx:40s} | [{q[:,j].min():.3f},{q[:,j].max():.3f}] | {m.sum()}")
