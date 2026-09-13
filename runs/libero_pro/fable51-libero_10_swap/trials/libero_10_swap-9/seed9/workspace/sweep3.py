import sys
sys.argv=['x']
exec(open('phase3.py').read().split('# mug-centre waypoints')[0])
CX=-0.15
seed=np.array([-1.004,1.296,-0.139,-0.984,1.374,1.163,-1.561])
for cy,psi in [(-0.40,35),(-0.40,30),(-0.38,30),(-0.37,28),(-0.36,25),(-0.33,20),(-0.31,15),(-0.29,10),(-0.27,5),(-0.25,0),(-0.24,0)]:
    h,q=hand_for(cy,psi,Z_IN)
    s=ik_near(r,h,q,seed,tries=8)
    if s is None: print(cy,psi,'IK fail'); continue
    dc=door_clearance(s); print(cy,psi,np.round(h,3),np.round(s,3),'door %.3f %s'%dc, flush=True)
