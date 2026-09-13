from rlib import *
from scene import *
r = Robot("j2t")
sc = Scene(r.node)
sc.publish(objects(drawer_front=-0.105, bowl=(-0.075, -0.17), with_bowl=True))
r.report()
# compact posture with j2=0.7: hand high near base
for q in [np.array([0.0, 0.7, 0.0, -2.6, 0.0, 3.3, 0.785]), np.array([0.0, 0.9, 0.0, -2.6, 0.0, 3.5, 0.785])]:
    p, R = r.fk(q); print("fk", np.round(p,3), np.round(R[:,2],2), sc.check(ARM, q, 0.0))
q1 = np.array([0.0, 0.7, 0.0, -2.6, 0.0, 3.3, 0.785])
q_home = np.array([0.0, 0.2, 0.0, -2.2, 0.0, 2.4, 0.785])
r.move_q(q_home); r.report()
r.move_q(q1); r.report()
r.move_q(q_home); r.report()
