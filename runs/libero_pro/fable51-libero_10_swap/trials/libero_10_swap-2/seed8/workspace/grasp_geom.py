"""Shared geometry for the diagonal, downward-tilted horizontal grasp."""
import numpy as np
from scipy.spatial.transform import Rotation as Rot

TILT = np.deg2rad(20.0)              # approach pitched below horizontal
DIR_H = np.array([0.70, -0.71, 0.0]); DIR_H /= np.linalg.norm(DIR_H)  # horizontal approach dir (+x,-y)
Z_H = np.array([DIR_H[0] * np.cos(TILT), DIR_H[1] * np.cos(TILT), -np.sin(TILT)])  # hand z (palm->tips)
Y_H = np.array([-DIR_H[1], DIR_H[0], 0.0])  # finger axis, horizontal, perpendicular
X_H = np.cross(Y_H, Z_H)
R_HAND = np.column_stack([X_H, Y_H, Z_H])
Q_HAND = Rot.from_matrix(R_HAND).as_quat()  # x,y,z,w

SHORT = 0.015   # TCP stops this far short of the pot axis along the approach


def tcp_for(axis_xy, z_tcp, back=0.0):
    """TCP position for grasping a vertical axis at axis_xy with pads at z_tcp;
    back>0 retreats along -Z_H."""
    p = np.array([axis_xy[0], axis_xy[1], z_tcp]) - SHORT * DIR_H
    return p - back * Z_H


if __name__ == "__main__":
    print("Z_H", np.round(Z_H, 4), "Y_H", np.round(Y_H, 4), "X_H", np.round(X_H, 4))
    print("Q_HAND", np.round(Q_HAND, 5))
    print("pre", np.round(tcp_for((-0.0443, -0.229), 0.972, 0.20), 4))
    print("final", np.round(tcp_for((-0.0443, -0.229), 0.972), 4))
