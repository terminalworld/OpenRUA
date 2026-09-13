import sys; sys.path.insert(0,"/workspace")
import numpy as np
from wlib import start
w=start()
OFF=0.1034
q=[float(x) for x in sys.argv[1].split(",")]; secs=float(sys.argv[2])
for i in range(3):
    code,err=w.move(q,secs)
    if err<0.01: break
T=w.fk_pose(); print("tcp now:", np.round(T[:3,3]+OFF*T[:3,2],4), flush=True)
w.destroy_node()
