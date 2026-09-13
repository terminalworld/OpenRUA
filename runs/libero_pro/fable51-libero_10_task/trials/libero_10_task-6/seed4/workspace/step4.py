from common import *
r = Robot()
print("lift"); r.go_tcp_line([*GRASP_XY, 0.545], [*GRASP_XY, 0.72], TOP, 3.0, n=2)
print("fingers", r.fingers())
