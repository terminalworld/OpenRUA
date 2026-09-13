#!/usr/bin/env python3
"""pose2.py tx ty tz  ax ay az  fx fy fz -> link8 pose (world) for TCP at t, hand approach axis a (points from hand
toward fingertips), finger-opening axis f (perpendicular to a). Fingertip offset 0.1175 m."""
import sys, numpy as np
from scipy.spatial.transform import Rotation as R
v=list(map(float,sys.argv[1:10])); t=np.array(v[0:3]); a=np.array(v[3:6]); f=np.array(v[6:9])
a=a/np.linalg.norm(a); f=f-np.dot(f,a)*a; f=f/np.linalg.norm(f); x=np.cross(f,a)
hand=R.from_matrix(np.column_stack([x,f,a]))
flange=t-0.1175*a
q=(hand*R.from_quat([0,0,0.3827,0.9239])).as_quat()
print(" ".join(f"{v:.4f}" for v in [*flange,*q]))
