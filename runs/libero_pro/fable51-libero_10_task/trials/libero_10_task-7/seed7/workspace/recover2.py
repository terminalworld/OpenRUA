from robot import *
r = Robot()
Q = Robot.quat_topdown(90)
q_home = np.array([0, -0.161037389, 0, -2.44459747, 0, 2.2267522, 0.7853981633974483])
q1 = r.ik_world([-0.224, -0.126, 0.64], Q, seed=q_home, attempts=1)
log("q1", q1.round(3), "current", r.arm_q().round(3))
ok = r.move_q(q1, 5.0)
log("tcp", r.fk_world()[0].round(3), "fingers", r.fingers())
