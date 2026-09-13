import numpy as np, sys
cam = sys.argv[1] if len(sys.argv) > 1 else 'agentview'
P=np.load(f'{cam}_world.npy'); x,y,z=P[...,0],P[...,1],P[...,2]
def fit(xs, ys):
    A=np.c_[2*xs,2*ys,np.ones(len(xs))]; b=xs**2+ys**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; return c[0],c[1],np.sqrt(c[2]+c[0]**2+c[1]**2)
for name,(y0,y1,handle_sign) in {'white/left':(-0.42,-0.18,+1),'yellow/right':(0.18,0.42,-1)}.items():
    reg=np.isfinite(z)&(x>-0.2)&(x<0.15)&(y>y0)&(y<y1)
    top=np.percentile(z[reg&(z>0.5)],99)
    rim=reg&(z>top-0.012)
    xs,ys=x[rim],y[rim]
    cx,cy,r=fit(xs,ys)
    # refit excluding handle side
    sel = (ys-cy)*handle_sign < r*0.3
    cx,cy,r=fit(xs[sel],ys[sel])
    plate=reg&(z>0.44)&(z<0.47)
    print(f'{name}: mug top z={top:.3f} rim center=({cx:.3f},{cy:.3f}) r={r:.3f}; plate pts x[{x[plate].min():.3f},{x[plate].max():.3f}] y[{y[plate].min():.3f},{y[plate].max():.3f}] plate ctr~({(x[plate].min()+x[plate].max())/2:.3f},{(y[plate].min()+y[plate].max())/2:.3f})')
