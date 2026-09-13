"""Pick the alphabet soup can: pre-grasp above, descend, close, lift."""
import sys
from rob import *

SOUP = np.array([-0.217, -0.135])
DOWN = [1.0, 0.0, 0.0, 0.0]  # hand z down, fingers close along world y
r = Robot("pick_soup")
q0 = r.arm_q()

print("== pre-grasp above soup")
q_pre = r.ik_tcp_world([*SOUP, 0.62], DOWN, seed=q0)
assert q_pre is not None
assert r.move_q(q_pre, 4.0)
print("  tcp now", r.tcp_world()[0].round(3))

print("== descend to grasp height")
q_g = r.ik_tcp_world([*SOUP, 0.462], DOWN, seed=q_pre)
assert q_g is not None
assert r.move_q(q_g, 3.0)
print("  tcp now", r.tcp_world()[0].round(3), "wrench", r.wrench())

print("== close")
gap = r.gripper(0.0)
print("  gap after close", gap)

print("== lift")
q_up = r.ik_tcp_world([*SOUP, 0.72], DOWN, seed=q_g)
assert q_up is not None
assert r.move_q(q_up, 3.0)
print("  tcp now", r.tcp_world()[0].round(3), "gap", r.finger_gap())
