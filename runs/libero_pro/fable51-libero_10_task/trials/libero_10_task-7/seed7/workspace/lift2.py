from robot import *
r = Robot()
pos, quat = r.fk_world()
r.move_line([pos[0], pos[1], 0.70], quat, speed=0.04, max_jump=0.6)
log("tcp", r.fk_world()[0].round(3), "fingers", r.fingers())
