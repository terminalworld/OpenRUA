import sys; sys.path.insert(0,"/workspace")
import numpy as np
from wlib import start
w=start()
home=[0.0,-0.161,0.0,-2.4446,0.0,2.2268,0.7854]
for i in range(3):
    code,err=w.move(home,3.5)
    if err<0.01: break
T=w.fk_pose(); print("hand:", np.round(T[:3,3],4))
w.destroy_node()
