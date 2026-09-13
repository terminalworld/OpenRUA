from robot import *
import sys
r = Robot()
x, y, z = map(float, sys.argv[1:4])
Q = Robot.quat_topdown(float(sys.argv[4]) if len(sys.argv) > 4 else 90)
q = r.move_world([x, y, z], Q, 4.0)
log("tcp", r.fk_world()[0].round(3), "q", r.arm_q().round(3))
