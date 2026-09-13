import sys, numpy as np
cam = sys.argv[1]; x0,x1,y0,y1 = map(float, sys.argv[2:6]); step = float(sys.argv[6]) if len(sys.argv)>6 else 0.01
P=np.load(f'{cam}_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]>0.903)&(P[:,2]<1.10)&(P[:,0]>x0)&(P[:,0]<x1)
Q=P[m]; print(len(Q))
ys=np.arange(y0,y1,step)
print('       y:', ' '.join(f"{abs(y):5.3f}"[1:] for y in ys))
for xa in np.arange(x0,x1,step):
    s=Q[(Q[:,0]>xa)&(Q[:,0]<xa+step)]
    if len(s)==0: continue
    row=[]
    for ya in ys:
        t=s[(s[:,1]>ya)&(s[:,1]<ya+step)]
        row.append(f"{(t[:,2].max()-0.9)*100:5.1f}" if len(t) else '    .')
    print(f"x {xa:.3f}:", ' '.join(row))
