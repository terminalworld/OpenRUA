import numpy as np
from rob import *

TARGET = np.array([-0.432, -0.153])
THETA = np.radians(-90)  # fingers close along world x -> book long axis along y

r = Rob()
print("transport high", flush=True)
r.move_tcp([TARGET[0], TARGET[1], 1.28], THETA, seconds=5.0)
print("fingers", r.fingers(), flush=True)
print("lower to TCP 1.15", flush=True)
r.move_tcp([TARGET[0], TARGET[1], 1.15], THETA, seconds=3.0)
print("fingers", r.fingers(), flush=True)
print("DONE", flush=True)
