from rob import *
r = Robot('s1')
s=np.sqrt(0.5)
q_view45 = quat_from_axes([0,s,s],[1,0,0],[0,s,-s])
q0 = r.arm_q()
qa = r.ik([-0.155,-0.40,1.32], q_view45, seed=q0)
print('via q', np.round(qa,3)); print('bad A:', path_check(r, q0, qa, verbose=False))
qb = r.ik([-0.155,-0.50,1.05], q_view45, seed=qa)
print('target q', np.round(qb,3)); print('bad B:', path_check(r, qa, qb, verbose=False))
import sys
if '--go' in sys.argv:
    r.move_q(qa, 4.0)
    r.move_q(qb, 3.0)
    print('fk', r.fk())
