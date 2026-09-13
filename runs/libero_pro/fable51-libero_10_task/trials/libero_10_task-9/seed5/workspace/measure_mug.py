import subprocess, numpy as np, sys
def measure(cam='agentview', box=(-0.25,-0.085,-0.42,-0.12), quiet=False):
    subprocess.run(['python3','tools/perception/cam_snap.py',f'/{cam}/depth/image_raw',f'{cam}_depth.png'],check=True,capture_output=True)
    subprocess.run(['python3','geo.py','cloud',cam],check=True,capture_output=True)
    P=np.load(f'{cam}_cloud.npy').reshape(-1,3)
    x0,x1,y0,y1=box
    m=np.isfinite(P).all(1)&(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]>0.905)&(P[:,2]<1.06)
    Q=P[m]
    if not quiet:
        for z0 in np.arange(0.905,1.06,0.02):
            s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.02)]
            if len(s): print(round(z0,3),len(s),'x',np.round([s[:,0].min(),s[:,0].max(),s[:,0].mean()],3),'y',np.round([s[:,1].min(),s[:,1].max(),s[:,1].mean()],3))
    return Q
if __name__=='__main__':
    measure(*(sys.argv[1:2] or ['agentview']))
