import sys
from robot import *
r = Robot("resend")
q = [float(v) for v in sys.argv[1].split(",")]; secs = float(sys.argv[2]) if len(sys.argv)>2 else 3.0
print("before", np.round(r.arm_q(),3))
code, err = r.move_joints([q], secs)
print("code", code, "max joint err", round(err,4), "now", np.round(r.arm_q(),3))
tcp, quat = r.tcp(); print("tcp now", np.round(tcp,4), np.round(quat,4))
