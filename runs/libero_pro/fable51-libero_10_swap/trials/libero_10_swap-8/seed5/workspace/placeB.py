from lib import *
r = Robot("placeB")
def tilted(yaw, pitch):
    return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_quat(down_quat(yaw))).as_quat()
POT_DROP = 0.139
tx, ty = 0.185, -0.015   # pot A landed ~9 mm short in x; compensate
pitch = -10
Qp = tilted(180, pitch)
h = Rot.from_quat(Qp).as_matrix()[:, 2]
base_z = 0.930 + 0.004 + 0.0335*np.sin(np.radians(10))
tcp_rel = np.array([tx, ty, base_z]) - POT_DROP*h
print("release TCP", np.round(tcp_rel,4))
print("-- traverse")
q = r.move_tcp((tcp_rel[0], tcp_rel[1], 1.22), Qp, 5)
if q is None:
    raise SystemExit("traverse IK failed")
print("gap", r.finger_gap())
print("-- descend")
r.move_tcp((tcp_rel[0], tcp_rel[1], 1.13), Qp, 3)
r.move_tcp(tcp_rel, Qp, 3)
print("-- release"); r.gripper(0.04)
print("-- retreat"); r.move_tcp((tcp_rel[0], tcp_rel[1], 1.20), Qp, 3)
