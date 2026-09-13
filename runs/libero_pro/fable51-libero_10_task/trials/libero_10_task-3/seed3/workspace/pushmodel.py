import numpy as np, math
import collide
from collide import OBST
def hand_points_closed(Th):
    pts = []
    for x in (-0.03, 0.03):
        for y in np.linspace(-0.10, 0.10, 9):
            for z in (-0.04, 0.0, 0.035, 0.07):
                pts.append([x, y, z, 1])
    for x in (-0.012, 0.012):
        for y in (-0.01, 0.0, 0.01):
            for z in (0.075, 0.09, 0.105):
                pts.append([x, y, z, 1])
    return (Th @ np.array(pts).T).T[:, :3]
collide.hand_points = hand_points_closed
OBST['tophandle'] = ([-0.05, 0.19, 1.005], [0.06, 0.23, 1.11])
OBST['cabinet'] = ([-0.14, 0.24, 0.90], [0.145, 0.44, 1.14])
OBST['lintel'] = ([-0.14, 0.228, 0.99], [0.145, 0.44, 1.14])
ign = ('drawer','bottle')
def extras(yp):
    return {'bar': ([-0.045, yp-0.033, 0.938], [0.055, yp-0.009, 0.962]),
            'postL': ([-0.037, yp-0.02, 0.938], [-0.023, yp, 0.962]),
            'postR': ([0.028, yp-0.02, 0.938], [0.043, yp, 0.962]),
            'paneltop': ([-0.11, yp, 0.983], [0.12, yp+0.02, 0.99])}
