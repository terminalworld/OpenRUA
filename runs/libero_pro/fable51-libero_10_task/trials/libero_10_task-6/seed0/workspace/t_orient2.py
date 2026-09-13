from robot import *
r = Robot("orient2")
def tq(yaw, pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
for yaw in [0, 90]:
    q = r.ik([-0.19, -0.05, 0.72], tq(yaw, 0))
    if q is None: print("IK fail"); continue
    pos, quat = r.fk(q); R = Rot.from_quat(quat).as_matrix()
    print(f"yaw {yaw}: hand y axis (finger closing dir) in world:", R[:,1].round(3), " z:", R[:,2].round(3), " tcp", np.round(r.tcp(q)[0],4))
