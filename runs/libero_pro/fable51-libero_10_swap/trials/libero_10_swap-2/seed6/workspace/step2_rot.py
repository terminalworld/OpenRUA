"""Rotate j7 by a delta while holding, report progress."""
import sys, numpy as np, pk
ROT = float(sys.argv[1]); secs = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
r = pk.Robot("rot")
q = r.joints(); print("start q", np.round(q, 3), "fingers", np.round(r.fingers(), 4))
q2 = q.copy(); q2[6] += ROT
r.move_joints([q2], secs)
q3 = r.joints()
print("j7 moved", round(q3[6] - q[6], 4), "of", ROT, "fingers", np.round(r.fingers(), 4), "wrench", np.round(r.wrench()[1], 2))
print("DONE")
