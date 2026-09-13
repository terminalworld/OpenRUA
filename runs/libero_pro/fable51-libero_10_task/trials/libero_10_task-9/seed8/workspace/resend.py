from lib import *
import sys
r = Robot("resend")
tgt = [float(v) for v in sys.argv[1].split(",")]; T=float(sys.argv[2])
q0 = np.array(r.arm_q()); print("start diff", np.round(np.array(tgt)-q0,3))
r.move([tgt],[T])
q1 = np.array(r.arm_q()); print("end diff", np.round(np.array(tgt)-q1,3))
r.report("after")
