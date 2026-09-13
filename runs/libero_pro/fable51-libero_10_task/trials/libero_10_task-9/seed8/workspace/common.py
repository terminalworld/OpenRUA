"""Shared bits for the white-mug task: hand orientation, scene setup."""
import numpy as np
from ctl import *

PH = np.radians(30)
YDX, YDY = 0.126, -0.013  # yellow mug shift after pushing it


def hand_R(psi, ph=PH):
    """Hand approaching horizontally along direction psi (yaw), pitched ph down; fingers close perpendicular to d."""
    d = np.array([np.cos(psi), np.sin(psi), 0])
    z = np.cos(ph) * d + np.array([0, 0, -np.sin(ph)])
    y = np.array([-np.sin(psi), np.cos(psi), 0]); x = np.cross(y, z)
    return R_from_axes(x, y, z)


def scene_no_white(sc):
    sc.publish(world_objects(with_white_mug=False) + yellow_objects(dx=YDX, dy=YDY), remove=["white_mug", "white_handle"])


def attach_hanging_mug(sc, lo=(-0.127, -0.054, 0.071), hi=(0.045, 0.05, 0.246)):
    sc.attach("carried_mug", np.array(lo), np.array(hi))
