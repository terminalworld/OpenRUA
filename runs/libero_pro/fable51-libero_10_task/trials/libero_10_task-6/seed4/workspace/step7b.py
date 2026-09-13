from common import *
import common
r = Robot()
Q2 = quat_mul(quat_z(math.radians(11)), TOP)
print("re-orient above pudding"); r.go_tcp([*PUD, 0.56], Q2, 3.0)
