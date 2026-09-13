from moka_plan import *
m = Mover("moka_a2")
q=m.arm_q(); print("q now", np.round(q,3))
target=[0.08,0.77,-0.23,-2.23,-2.43,1.79,0.81]
print("resend pre q"); m.goto_q(target, 4.0)
q=m.arm_q(); print("q now", np.round(q,3), "target", target)
p,qq=m.fk_world(); print("hand", np.round(p,4), "quat err", 1-abs(np.dot(qq,Q_GRASP)))
print("wrench", wrench(m))
