import numpy as np
from rob import *

# book centre sits 1.8 mm toward -y of the TCP; want book centre y = -0.1525
TCP_XY = np.array([-0.432, -0.1507])
THETA = np.radians(-90)

r = Rob()
print("lower to TCP 1.10", flush=True)
r.move_tcp([TCP_XY[0], TCP_XY[1], 1.10], THETA, seconds=2.5)
print("lower to TCP 1.065 (fingertips 7 mm above back wall top)", flush=True)
r.move_tcp([TCP_XY[0], TCP_XY[1], 1.065], THETA, seconds=2.0)
print("fingers", r.fingers(), flush=True)
print("DONE", flush=True)
