from common import *
r = Robot()
DEST = np.array([PLATE[0], 0.21])
print("lift"); r.go_tcp_line([*PUD, 0.445], [*PUD, 0.60], PUD_Q, 2.5, n=2)
print("fingers", r.fingers())
print("to dest above"); r.go_tcp([*DEST, 0.60], PUD_Q, 4.0)
print("fingers", r.fingers())
