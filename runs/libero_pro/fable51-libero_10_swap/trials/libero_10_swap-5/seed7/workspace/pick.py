"""Stage A: pick up the book (top-down grasp across its thickness) and lift."""
from lib import *

r = Robot()
yaw_b = np.radians(-8.7)
bc = np.array([-0.096, -0.021])
Rg = down_R(yaw_b)

print("fingers before:", r.fingers())
r.gripper(GRIP["open_m"])

q = r.move_tcp([bc[0], bc[1], 1.20], Rg, seconds=4.0)
assert q is not None
# descend in two steps, seeded for branch continuity
q = r.move_tcp([bc[0], bc[1], 1.08], Rg, seconds=2.0, seed=q)
q = r.move_tcp([bc[0], bc[1], 0.99], Rg, seconds=2.0, seed=q)
assert q is not None

r.gripper(GRIP["closed_m"])
f = r.fingers()
print("fingers after close:", f, "gap ~", f[0] * 2 if f[0] > 0 else f[0] - f[1])

q = r.move_tcp([bc[0], bc[1], 1.30], Rg, seconds=3.0, seed=q)
print("fingers after lift:", r.fingers())
print("arm q:", np.round(r.arm_q(), 4))
