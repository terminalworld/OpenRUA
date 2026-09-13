from lib import *
r = Robot("placeA")
def tilted(yaw, pitch):
    return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_quat(down_quat(yaw))).as_quat()
POT_DROP = 0.139   # knob-grasp TCP to pot base
tx, ty = 0.178, 0.075  # desired pot base center
pitch = -10
h = Rot.from_quat(tilted(180, pitch)).as_matrix()[:, 2]
print("hand z axis", np.round(h,4))
base_z = 0.930 + 0.004 + 0.0335*np.sin(np.radians(10))
tcp_rel = np.array([tx, ty, base_z]) - POT_DROP*h
print("release TCP", np.round(tcp_rel,4))
print("gap", r.finger_gap())
print("-- lift")
p, _ = r.tcp_world()
r.move_tcp((p[0], p[1], 1.25), down_quat(90), 3)
print("-- traverse")
q = r.move_tcp((tcp_rel[0], tcp_rel[1], 1.22), tilted(180, pitch), 5)
if q is None:
    q = r.move_tcp((tcp_rel[0]-0.03, tcp_rel[1], 1.22), tilted(180, pitch), 5)
print("gap", r.finger_gap())
