from robot import *
import sys
r = Robot()
z = float(sys.argv[1]) if len(sys.argv) > 1 else 0.70
if r.fingers()[0] < 0.035: r.gripper(GRIP["open_m"])
pos, quat = r.fk_world()
r.move_line([pos[0], pos[1], z], quat, speed=0.05, max_jump=0.6)
log("tcp", r.fk_world()[0].round(3), "q", r.arm_q().round(3))
