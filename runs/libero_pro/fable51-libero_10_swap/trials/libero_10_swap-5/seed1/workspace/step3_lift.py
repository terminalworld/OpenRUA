import numpy as np
from rob import *

BOOK = np.array([-0.122, 0.0195])
THETA = np.radians(158.28 - 180)

r = Rob()
print("lift", flush=True)
r.move_tcp([BOOK[0], BOOK[1], 1.30], THETA, seconds=3.0)
print("fingers", r.fingers(), flush=True)
print("DONE", flush=True)
