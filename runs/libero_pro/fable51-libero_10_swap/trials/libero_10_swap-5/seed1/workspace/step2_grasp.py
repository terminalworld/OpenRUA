import numpy as np
from rob import *

BOOK = np.array([-0.122, 0.0195])
BOOK_TOP = 1.023  # consensus of side/front/agent cameras
THETA = np.radians(158.28 - 180)

r = Rob()
print("open", flush=True)
r.gripper(GRIP["open_m"])
print("pre-grasp hover", flush=True)
r.move_tcp([BOOK[0], BOOK[1], BOOK_TOP + 0.06], THETA, seconds=3.0)
print("descend to TCP 3 cm below book top", flush=True)
r.move_tcp([BOOK[0], BOOK[1], BOOK_TOP - 0.03], THETA, seconds=2.5)
print("close", flush=True)
f = r.gripper(GRIP["closed_m"])
print("lift", flush=True)
r.move_tcp([BOOK[0], BOOK[1], 1.28], THETA, seconds=3.0)
print("fingers after lift", r.fingers(), flush=True)
print("DONE", flush=True)
