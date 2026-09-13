#!/usr/bin/env python3
"""pose.py tcp_x tcp_y tcp_z tilt_deg [yaw180=1] -> prints 'x y z qx qy qz qw' for panda_link8 (world).
Hand points down, fingers open along world Y; tilt>0 leans the approach toward -x (base side)."""
import sys, numpy as np
from scipy.spatial.transform import Rotation as R
tx,ty,tz,tilt=map(float,sys.argv[1:5]); yaw180=int(sys.argv[5]) if len(sys.argv)>5 else 1
q0 = R.from_quat([0,1,0,0]) if yaw180 else R.from_quat([1,0,0,0])     # hand Z down
hand = R.from_rotvec(np.deg2rad(tilt)*np.array([0,1,0])) * q0
zh = hand.apply([0,0,1])                                             # hand approach axis in world
flange = np.array([tx,ty,tz]) - 0.1175*zh
l8 = hand * R.from_quat([0,0,0.3827,0.9239])                         # hand = l8 * (0,0,-0.3827,0.9239)
q = l8.as_quat()
print(" ".join(f"{v:.4f}" for v in [*flange,*q]))
