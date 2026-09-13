from robot import *
r = Robot()
# park above the pudding's neighbourhood but high, fingers closing along y
r.move_tcp([-0.08, 0.10, 0.80], down_quat(0), 4.0)
