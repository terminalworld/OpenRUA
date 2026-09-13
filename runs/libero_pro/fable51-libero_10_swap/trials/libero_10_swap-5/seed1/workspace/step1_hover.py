import numpy as np
from rob import *

BOOK = np.array([-0.122, 0.0195])
BOOK_TOP = 1.07
THETA = np.radians(158.28 - 180)  # fingers close across the book's 3 cm thickness

r = Rob()
print("open gripper", flush=True)
r.gripper(GRIP["open_m"])
print("hover above book", flush=True)
r.move_tcp([BOOK[0], BOOK[1], BOOK_TOP + 0.10], THETA, seconds=4.0)
print("q", np.round(r.arm_q(), 3), flush=True)
print("DONE", flush=True)
