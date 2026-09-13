import sys
from robot import *
plan=np.load('plan3.npy',allow_pickle=True).item(); plan.update(np.load('plan4.npy',allow_pickle=True).item()); plan.update(np.load('plan5.npy',allow_pickle=True).item()); plan.update(np.load('plan6.npy',allow_pickle=True).item())
r=Robot()
def run(qs,vmax=0.15):
    # one goal per waypoint; bridge velocity limit ~0.23 rad/s, stay well under it
    for q in qs:
        dt=max(np.abs(q-r.arm_q()).max()/vmax,0.6)
        code=r.move_q(q,dt)
        if code!=0: print('ABORT step'); return code
    return 0
args=sys.argv[1:]
vmax=0.1
for a in args:
    if a.startswith('v='): vmax=float(a[2:]); continue
    if a=='open': r.gripper(0.04); continue
    if a=='close': r.gripper(0.0); continue
    if '[' in a:
        name,sl=a.split('['); qs=eval('plan[name]['+sl)
    else: qs=plan[a]
    print('step',a,len(qs),'pts')
    if run(qs,vmax)!=0: break
q=r.arm_q(); p,R=r.tcp(q)
print('q',np.round(q,3)); print('tcp',np.round(p,4)); print('fingers',r.fingers()); print('wrench',np.round(r.wrench(),2))
