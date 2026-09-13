from robot import *
r = Robot()
BX, BY, BZ = 0.01, 0.26, 0.72
pos, quat = r.fk_world()
ok = r.move_line([pos[0], pos[1], 0.70], quat, speed=0.04, max_jump=0.6)
log("lifted", ok, "tcp", r.fk_world()[0].round(3), "fingers", r.fingers())
ok = r.move_line([BX, BY, BZ], quat, speed=0.06, step=0.03, max_jump=0.6)
if not ok:
    log("line failed; falling back to IK move")
    r.move_world([BX, BY, BZ], quat, 5.0)
log("over basket tcp", r.fk_world()[0].round(4), "fingers", r.fingers())
r.gripper(GRIP["open_m"])
log("released; fingers", r.fingers())
r.move_line([BX, BY, 0.85], quat, speed=0.06, step=0.03, max_jump=0.6)
log("retreated tcp", r.fk_world()[0].round(3))
