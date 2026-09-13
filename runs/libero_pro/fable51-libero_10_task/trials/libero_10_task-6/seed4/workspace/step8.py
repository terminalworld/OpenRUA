from common import *
r = Robot()
print("descend"); r.go_tcp_line([*PUD, 0.56], [*PUD, 0.445], PUD_Q, 3.0, n=3)
print("close"); r.gripper(0.0)
q,_ = r.joints(); r.move([q],[0.5]); print("fingers settled", r.fingers())
