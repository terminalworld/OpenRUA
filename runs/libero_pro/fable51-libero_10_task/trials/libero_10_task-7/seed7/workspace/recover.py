from robot import *
r = Robot()
pos, quat = r.fk_world()
log("tcp", pos.round(3))
r.move_line([pos[0], pos[1], 0.66], quat, speed=0.05)
log("fingers", r.fingers())
