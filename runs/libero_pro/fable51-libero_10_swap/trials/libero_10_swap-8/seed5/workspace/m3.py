from lib import *
import sys
r = Robot("m3")
q = [0.197,-0.199,-0.575,-2.231,-0.122,2.06,-1.882]
r.move_q(q, float(sys.argv[1]) if len(sys.argv)>1 else 8)
print("q now", np.round(r.arm_q(),3))
pos, quat = r.fk_world(); print("hand", np.round(pos,4), "euler", np.round(Rot.from_quat(quat).as_euler("xyz", degrees=True),1))
