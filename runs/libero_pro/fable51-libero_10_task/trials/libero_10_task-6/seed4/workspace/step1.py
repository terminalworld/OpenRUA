from common import *
r = Robot()
print("pre-grasp above rim"); q = r.go_tcp([*GRASP_XY, 0.65], TOP, 4.0)
