from robot import *
r = Robot()
BX, BY, BZ = 0.01, 0.26, 0.74
Q45 = Robot.quat_topdown(45)
pos, _ = r.fk_world()
log("rotate in place to yaw 45")
assert r.move_world(pos, Q45, 3.0) is not None
log("line to basket")
ok = r.move_line([BX, BY, BZ], Q45, speed=0.06, step=0.03, max_jump=0.6)
log("over basket ok", ok, "tcp", r.fk_world()[0].round(4), "fingers", r.fingers())
r.gripper(GRIP["open_m"])
log("released; fingers", r.fingers())
r.move_line([BX, BY, 0.85], Q45, speed=0.06, step=0.03, max_jump=0.6)
log("retreated tcp", r.fk_world()[0].round(3))
