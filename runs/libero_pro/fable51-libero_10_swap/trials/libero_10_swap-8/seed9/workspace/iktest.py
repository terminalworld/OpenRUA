import numpy as np, sys
from rob import *
r = Robot("iktest")
q0 = r.arm_q(); print("q0", q0)
p, quat = r.fk_base(); print("hand world", p, quat, "tcp world", r.tcp_world()[0])
# candidate downward orientations: fingers along world x
cands = {"fx+": (0.7071,0.7071,0,0), "fx-": (0.7071,-0.7071,0,0), "fy": (1,0,0,0), "fy2": (0,1,0,0)}
for name,(x,y,z) in {"potA":(-0.196,-0.200,0.97),"potB":(-0.035,0.238,0.97),"stove1":(0.165,-0.01,1.0),"stove2":(0.165,0.09,1.0),"stoveC":(0.21,0.04,1.0)}.items():
    for cn, quat in cands.items():
        sol = r.ik_tcp_world((x,y,z), quat, seed=q0)
        ok = sol is not None
        chk = ""
        if ok:
            tw, _ = r.tcp_world(sol); chk = f"fk-check tcp={np.round(tw,3)} q={np.round(sol,2)}"
        print(name, cn, "OK" if ok else "FAIL", chk, flush=True)
