import numpy as np
from rob import *

TCP_XY = np.array([-0.432, -0.1507])
THETA = np.radians(-90)

r = Rob()
print("open", flush=True)
r.gripper(GRIP["open_m"])
print("retreat up", flush=True)
r.move_tcp([TCP_XY[0], TCP_XY[1], 1.25], THETA, seconds=3.0)
print("move aside so cameras see the caddy", flush=True)
r.move_tcp([-0.25, 0.10, 1.30], np.radians(0), seconds=4.0)
print("DONE", flush=True)
