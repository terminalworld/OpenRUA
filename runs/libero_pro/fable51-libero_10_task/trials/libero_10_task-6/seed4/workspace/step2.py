from common import *
r = Robot()
print("descend onto rim"); r.go_tcp_line([*GRASP_XY, 0.65], [*GRASP_XY, 0.545], TOP, 3.0, n=3)
print("fingers before", r.fingers())
