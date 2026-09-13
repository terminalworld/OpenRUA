import numpy as np
from rob import R_from_axes, quat_from_axes
# mug in hand coords (pinch at +x rim point, pads 1.5cm below rim)
RIM_C = np.array([0, -0.051, 0.078]); BASE_C = np.array([0, -0.051, 0.178]); RB = 0.033; RR = 0.051
DOOR_Y = -0.325; FLOOR = 0.94; CEIL = 1.095; TOP = 1.107
def pose(theta_deg, mug_x, final_center_y=None, Lz=0.943, H_y=None):
    th = np.radians(theta_deg)
    z_h = np.array([0, np.sin(th), -np.cos(th)]); R = R_from_axes(z_h, (1, 0, 0)); x_h = R[:, 0]
    # lowest base point L = H + R@BASE_C - RB*x_h
    off_L = R @ BASE_C - RB * x_h
    if H_y is None:
        # housing +y corner (finger-base end, upper face) at door plane
        corner = 0.058 * z_h + 0.03 * x_h
        H_y = DOOR_Y - corner[1]
    H = np.array([mug_x + 0.051, H_y, Lz - off_L[2]])
    L = H + off_L
    info = dict(H=H, L=L, final_center_y=L[1] + RB,
                upper_corner=H + 0.03 * x_h, lower_corner=H + 0.058 * z_h + 0.03 * x_h,
                rim_plus_y=H + R @ RIM_C + RR * x_h, base_plus_y=H + R @ BASE_C + RB * x_h)
    return quat_from_axes(z_h, (1, 0, 0)), info
if __name__ == "__main__":
    for th in (30, 33, 36):
        q, i = pose(th, -0.17)
        print(th, {k: np.round(v, 3) for k, v in i.items()})
