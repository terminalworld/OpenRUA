import numpy as np
from rob import *
r = Robot("iktest3")
def rot_y(th):
    c,s=np.cos(th),np.sin(th); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
seedA = np.array([0.03,1.13,-0.08,-0.95,0.09,2.25,1.52])
for base in [(1,0,0,0),(0,1,0,0)]:
    for pitch in [10,12,15]:
        q = R_to_quat(rot_y(np.radians(-pitch)) @ quat_to_R(*base))
        for y in [-0.012, 0.092]:
            for z in [1.055, 1.135]:
                for x in [0.15,0.16,0.17]:
                    sol = r.ik_tcp_world((x,y,z), q, seed=seedA, timeout=3)
                    ok = sol is not None
                    extra = ""
                    if ok:
                        tcp, fq = r.tcp_world(sol); R = quat_to_R(*fq)
                        extra = f"j={np.round(sol,2)} hz={np.round(R[:,2],2)}"
                    print(f"base={base} pitch={pitch} y={y} z={z} x={x}: {'OK' if ok else 'fail'} {extra}", flush=True)
