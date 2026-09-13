import numpy as np
from rob import *
r = Rob()
pos, q, tcp = r.fk_pose()
print("fk hand", pos.round(4), "q", np.round(q,4))
cur = np.array(r.arm_q())
# hypothesis A: IK expects world frame -> use fk pos directly (hand pose, so pass tcp)
solA = r.ik_tcp(*tcp, q=q)
print("A (world) sol", None if solA is None else np.round(solA,3), "diff", None if solA is None else np.abs(np.array(solA)-cur).max().round(3))
# hypothesis B: base frame -> subtract base offset
tcpB = tcp - BASE_W
solB = r.ik_tcp(*tcpB, q=q)
print("B (base) sol", None if solB is None else np.round(solB,3), "diff", None if solB is None else np.abs(np.array(solB)-cur).max().round(3))
r.shutdown()
