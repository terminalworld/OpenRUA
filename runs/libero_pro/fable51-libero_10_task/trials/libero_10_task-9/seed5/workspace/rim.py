import numpy as np, json
from geo import quat_R
D=np.load('robot0_eye_in_hand_depth.npy'); f=312.774
c=json.load(open('cams.json'))['robot0_eye_in_hand_optical_frame']
R=quat_R(*c['q']); T=np.array(c['t'])
vs,us=np.mgrid[0:480,0:640]
Z=D; X=(us-320)*Z/f; Y=(vs-240)*Z/f
P=np.stack([X,Y,Z],-1)@R.T+T   # world
wz=P[...,2]
obj=(wz>0.905)&(Z<0.4)&(vs<400)
hx=0.05-Y; hy=X; hz=Z
print('object px',obj.sum())
for lo,hi in [(-0.08,-0.06),(-0.06,-0.04),(-0.04,-0.02),(-0.02,0.0),(0.0,0.01),(0.01,0.02),(0.02,0.04)]:
    m=obj&(hy>=lo)&(hy<hi)
    if m.sum()==0: print(lo,hi,'none'); continue
    zmin=hz[m].min()
    mm=m&(hz<zmin+0.008)
    print(f'hy[{lo},{hi}] n={m.sum()} hz_min={zmin:.3f} front hx {hx[mm].min():.3f}..{hx[mm].max():.3f}  wz {wz[mm].min():.3f}..{wz[mm].max():.3f} wxy {P[mm][:,0].mean():.3f},{P[mm][:,1].mean():.3f}')
m=obj; i=np.argmax(hy[m]); print('max hy',round(hy[m].max(),4), 'at hx',round(hx[m][i],4),'hz',round(hz[m][i],4),'world',np.round(P[m][i],3))
