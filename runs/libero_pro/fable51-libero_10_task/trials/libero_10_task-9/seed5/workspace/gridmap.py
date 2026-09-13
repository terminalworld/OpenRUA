import numpy as np, sys
P=np.load(sys.argv[1]); x0,x1,y0,y1=map(float,sys.argv[2:6]); zmin=float(sys.argv[6]) if len(sys.argv)>6 else 0.905
cs=0.01
m=np.isfinite(P[...,2])&(P[...,2]>zmin)&(P[...,0]>=x0)&(P[...,0]<x1)&(P[...,1]>=y0)&(P[...,1]<y1)
Q=P[m]
nx=int(round((x1-x0)/cs)); ny=int(round((y1-y0)/cs))
G=np.full((ny,nx),np.nan)
ix=((Q[:,0]-x0)/cs).astype(int); iy=((Q[:,1]-y0)/cs).astype(int)
for a,b,z in zip(ix,iy,Q[:,2]):
    if np.isnan(G[b,a]) or z>G[b,a]: G[b,a]=z
print('      x:'+''.join(f'{x0+cs*i+cs/2:6.2f}' for i in range(nx)))
for b in range(ny-1,-1,-1):
    row=''.join('   .  ' if np.isnan(G[b,a]) else f'{(G[b,a]-0.90)*100:5.1f} ' for a in range(nx))
    print(f'y={y0+cs*b+cs/2:6.2f} '+row)
