"""Soup pick, take 2: straight-line descent with yaw fixed, close, lift."""
from rob import *

SOUP = np.array([-0.2117, -0.135])
r = Robot("pick_soup2")
print("tcp", r.tcp_world()[0].round(4), "gap", r.finger_gap())

print("== align above can at z=0.60")
assert r.move_line([*SOUP, 0.60], 0.0, seconds=3.0)
print("== straight descent to grasp height")
assert r.move_line([*SOUP, 0.462], 0.0, seconds=3.0, step=0.015)
print("  wrench", r.wrench())
print("== close")
gap = r.gripper(0.0)
print("== lift")
assert r.move_line([*SOUP, 0.72], 0.0, seconds=3.0)
print("  gap after lift", r.finger_gap(), "wrench", r.wrench())
