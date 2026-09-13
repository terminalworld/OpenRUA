"""Stage B: carry the book over the back compartment (yaw 90 deg so its long
axis runs along world y) and lower it until its bottom is just above the
wall tops. Stops there for a visual check."""
from lib import *

r = Robot()
R90 = down_R(np.pi / 2)
cx, cy = -0.4315, -0.1465          # back compartment interior centre
q1 = list(np.load("q1.npy"))

print("fingers:", r.fingers())
r.move(q1, 4.0)
q = r.move_tcp([cx, cy, 1.30], R90, seconds=2.0, seed=q1)
assert q is not None
prev = np.array(q)
for z in (1.27, 1.23):
    qn = r.ik([cx, cy, z], R90, seed=list(prev), tries=3)
    assert qn is not None, f"IK failed at z={z}"
    d = np.abs(np.array(qn) - prev).max()
    print(f"z={z}: max joint delta {d:.3f}")
    assert d < 0.6, "joint jump too large; aborting before motion"
    q = r.move_tcp([cx, cy, z], R90, seconds=2.0, seed=list(prev))
    prev = np.array(q)
np.save("q_prev.npy", prev)
print("fingers:", r.fingers())
