from lib import *
r = Robot("placeA2")
def tilted(yaw, pitch):
    return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_quat(down_quat(yaw))).as_quat()
tcp_rel = (0.1539, 0.075, 1.0767)
print("-- descend")
r.move_tcp((tcp_rel[0], tcp_rel[1], 1.13), tilted(180, -10), 3)
r.move_tcp(tcp_rel, tilted(180, -10), 3)
print("-- release")
r.gripper(0.04)
print("-- retreat")
r.move_tcp((tcp_rel[0], tcp_rel[1], 1.20), tilted(180, -10), 3)
