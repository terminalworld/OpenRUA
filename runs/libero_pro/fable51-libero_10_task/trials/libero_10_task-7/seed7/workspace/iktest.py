from robot import *
r = Robot()
pos, quat = r.fk_world(); q0 = r.arm_q()
print("q0", q0.round(3))
for i in range(4):
    q = r.ik_world(pos, quat, seed=q0, attempts=1)
    print("same pose:", None if q is None else (q.round(3), f"jump {np.abs(q-q0).max():.3f}"))
for i in range(3):
    q = r.ik_world(pos+[0,0,0.02], quat, seed=q0, attempts=1)
    print("+2cm:", None if q is None else (q.round(3), f"jump {np.abs(q-q0).max():.3f}"))
