from robot import *
r = Robot()
KX, KY = -0.224, -0.126
YAW = 90            # fingers open along world x (bottle's thin axis)
Q = Robot.quat_topdown(YAW)
BASKET = (0.01, 0.26)

f = r.fingers()
if f[0] < 0.035: r.gripper(GRIP["open_m"])
log("pre-grasp"); q1 = r.move_world([KX, KY, 0.64], Q, 4.0); assert q1 is not None
log("tcp", r.fk_world()[0].round(3))
log("descend"); q2 = r.move_world([KX, KY, 0.475], Q, 3.0, seed=q1); assert q2 is not None
log("tcp", r.fk_world()[0].round(3))
log("close"); f = r.gripper(GRIP["closed_m"])
log("lift"); q3 = r.move_world([KX, KY, 0.75], Q, 3.0, seed=q2); assert q3 is not None
log("fingers after lift", r.fingers())
log("over basket"); q4 = r.move_world([BASKET[0], BASKET[1], 0.78], Q, 4.0, seed=q3); assert q4 is not None
log("tcp", r.fk_world()[0].round(3))
log("release"); r.gripper(GRIP["open_m"])
log("retreat up"); q5 = r.move_world([BASKET[0], BASKET[1], 0.85], Q, 2.0, seed=q4)
log("DONE")
