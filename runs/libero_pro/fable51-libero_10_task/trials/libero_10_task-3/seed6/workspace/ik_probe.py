import math, numpy as np
from rb import Robot, R_quat
r = Robot("probe")
ang = math.radians(15); yh0 = np.array([math.cos(ang), -math.sin(ang), 0.0]); b = math.radians(20)
yh = math.cos(b)*yh0 + math.sin(b)*np.array([0,0,1.0]); zh = math.sin(b)*yh0 - math.cos(b)*np.array([0,0,1.0]); xh = np.cross(yh, zh)
q = R_quat(np.column_stack([xh, yh, zh]))
tcp = np.array([-0.179, 0.13, 1.02])
cur = r.arm_q()
seeds = {"cur": cur, "ready": [0,-0.785,0,-2.356,0,1.571,0.785], "ready_j7-": [0,-0.785,0,-2.356,0,1.571,-0.8], "ready_j7+": [0,-0.785,0,-2.356,0,1.571,2.3]}
for n, s in seeds.items():
    sol = r.ik(tcp, q, seed=s, at_tcp=True)
    if sol: print(n, np.round(sol,2), "maxdelta from cur", round(max(abs(a-b) for a,b in zip(sol,cur)),2))
