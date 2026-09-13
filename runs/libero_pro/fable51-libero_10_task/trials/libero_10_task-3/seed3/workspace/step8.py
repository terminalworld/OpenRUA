import numpy as np, math, sys
from panda import *
from collide import check, check_path
from geom import place_pose, report, bottle_pts
np.set_printoptions(precision=4, suppress=True)
r = Robot()
q0 = r.q(); print('q0', q0.round(3), 'tcp', fk_tcp(q0)[:3,3].round(3), 'fingers', r.fingers())
Rt = topdown(math.pi/2)
BX, BY = -0.044, 0.122
T1 = make_T([BX, BY, 1.20], Rt)
T2 = make_T([BX, BY, 1.048], Rt)      # neck grasp
T3 = make_T([BX, BY, 1.22], Rt)       # lift
T4 = make_T([-0.10, 0.06, 1.30], Rt)   # free space
Rp = place_pose(math.radians(50), math.radians(22), [0,0,0])[:3,:3]
T5 = make_T([-0.10, 0.06, 1.30], Rp)   # reoriented
T6 = make_T([-0.05, 0.14, 1.25], Rp)  # above drawer
T7 = make_T([-0.05, 0.14, 1.045], Rp) # release pose
ign = ('bottle','drawer','tophandle')
q1 = ik(T1, q0); q2 = ik(T2, q1); q3 = ik(T3, q2); q4 = ik(T4, q3)
# reorientation via slerp waypoints
qs5 = []; qp = q4
for s in np.linspace(0, 1, 8)[1:]:
    Ti = interp_T(T4, T5, s); qi = ik(Ti, qp); assert qi is not None, ('ik fail reorient', s); qs5.append(qi); qp = qi
q5 = qs5[-1]
q6 = ik(T6, q5); q7 = ik(T7, q6)
for n,q in (('q1',q1),('q2',q2),('q3',q3),('q4',q4),('q5',q5),('q6',q6),('q7',q7)):
    print(n, q.round(3), check(q, ignore=ign, verbose=(n in ('q5','q7'))))
print('paths', check_path([q0,q1], ignore=('bottle',)), check_path([q1,q2,q3], ignore=ign), check_path([q3,q4], ignore=ign),
      check_path([q4]+qs5, ignore=ign), check_path([q5,q6], ignore=ign), check_path([q6,q7], ignore=ign))
# geometric check of descent
for s in np.linspace(0,1,6):
    Ti = interp_T(T6, T7, s); rep = report(Ti, False, [-1,0,0]); print('descent s=%.1f min clr %.3f (%s)'%(s, min(rep.values()), min(rep,key=rep.get)))
rep = report(T7, True, [-1,0,0]); print('release (open) min clr %.3f (%s)'%(min(rep.values()), min(rep,key=rep.get)))
B = bottle_pts(T7,[-1,0,0]); print('bottle at release x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(B[:,0].min(),B[:,0].max(),B[:,1].min(),B[:,1].max(),B[:,2].min(),B[:,2].max()))
if '--go' in sys.argv:
    r.gripper(0.04)
    print(r.move_q(q1)); print(r.move_tcp(T2, n_wp=3)); print('tcp', r.tcp()[:3,3].round(4))
    f = r.gripper(0.0)
    if not (0.004 < f[0] < 0.012): print('GRASP LOOKS WRONG, stopping'); sys.exit(1)
    print(r.move_tcp(T3, n_wp=2)); print('fingers', r.fingers())
    print(r.move_q(q4)); print('fingers', r.fingers())
    print(r.move_traj_checked if False else '')
    # reorientation
    ts=[]; t=0; qp=q4
    for qi in qs5: t += max(np.abs(qi-qp).max()/0.15, 0.05)+0.2; ts.append(t); qp=qi
    print('reorient code', r.move_traj(qs5, ts)); print('q err', np.abs(r.q()-q5).max().round(4), 'fingers', r.fingers(), 'tcp', r.tcp()[:3,3].round(4))
    print(r.move_q(q6)); print('fingers', r.fingers())
    print(r.move_tcp(T7, n_wp=4)); print('tcp', r.tcp()[:3,3].round(4), 'fingers', r.fingers(), 'wrench', r.wrench().round(2))
    r.gripper(0.04)
    print(r.move_tcp(T6, n_wp=3)); print('tcp', r.tcp()[:3,3].round(4))
