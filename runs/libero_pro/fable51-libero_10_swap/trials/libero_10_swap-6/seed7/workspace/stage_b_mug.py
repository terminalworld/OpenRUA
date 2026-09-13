#!/usr/bin/env python3
"""Stage B: pick the white mug by its rim (side opposite the handle) and set it on the plate."""
from robot import *

MUG = np.array([-0.1175, -0.177])      # mug axis (world xy), rim fit from birdview
R_MUG = 0.0447                         # outer radius; wall ~5 mm thick
RIM_Z = 0.549
PLATE = np.array([0.141, 0.000])
PLATE_Z = 0.443                        # plate inner surface
OFF = np.array([0.0, -0.041])          # gripper centre sits on the -y wall (handle is +y)
GRASP_Z = 0.520                        # TCP 2.9 cm below the rim
MUG_BOTTOM_BELOW_TCP = (RIM_Z - 0.425) - (RIM_Z - GRASP_Z)   # 0.095
Q = down_quat(0)                       # fingers along world y (radial at the -y wall)
READY = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])

r = Robot()
print("== open gripper"); r.open()
g = MUG + OFF
print("== above mug wall"); assert r.move_tcp([*g, 0.65], Q)
print("== descend");        assert r.move_tcp_line([*g, 0.65], [*g, GRASP_Z], Q, steps=4)
p, _ = r.tcp(); print("   tcp now", np.round(p, 4))
print("== close");          gap = r.close()
print("   finger gap after close:", round(gap, 4), "(expect ~wall thickness, >0)")
print("== lift");           assert r.move_tcp_line([*g, GRASP_Z], [*g, 0.66], Q, steps=3)
print("   gap while lifted:", round(r.finger_gap(), 4))
dest = PLATE + OFF
print("== over plate");     assert r.move_tcp([*dest, 0.66], Q)
place_z = PLATE_Z + MUG_BOTTOM_BELOW_TCP + 0.006
print(f"== lower to place z={place_z:.3f}"); assert r.move_tcp_line([*dest, 0.66], [*dest, place_z], Q, steps=4)
print("== release");        r.open()
print("== retreat up");     assert r.move_tcp_line([*dest, place_z], [*dest, 0.68], Q, steps=3)
print("== back to ready");  r.move_joints(READY)
print("STAGE B DONE")
