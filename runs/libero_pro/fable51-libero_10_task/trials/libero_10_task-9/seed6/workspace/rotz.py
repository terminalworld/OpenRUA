"""Rotate hand about world z through the TCP by angle_deg (Cartesian, smooth)."""
import sys, numpy as np
from scipy.spatial.transform import Rotation as Rot
from rob import *; from plan import Planner
ang = np.radians(float(sys.argv[1])); n = int(sys.argv[2]) if len(sys.argv) > 2 else 8
r = Planner("rotz")
tcp, R0 = r.tcp()
targets = []
for k in range(1, n+1):
    Rk = Rot.from_rotvec([0, 0, ang*k/n]).as_matrix() @ R0
    targets.append((hand_pose_from_tcp(tcp, Rk), Rk))
traj, frac = r.cartesian(targets, avoid=False, step=0.005)
print("fraction", frac)
if traj is None or frac < 0.99: sys.exit("cartesian failed")
r.execute(traj)
p, R = r.tcp(); print("tcp", p.round(3), "finger axis", R[:,1].round(3), "approach", R[:,2].round(3), "fingers", np.round(r.fingers(),4))
