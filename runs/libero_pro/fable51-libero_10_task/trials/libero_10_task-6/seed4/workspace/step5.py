from common import *
r = Robot()
print("to above plate"); r.go_tcp([*(PLATE + [0, R_MUG]), 0.72], TOP, 4.0)
print("fingers", r.fingers())
