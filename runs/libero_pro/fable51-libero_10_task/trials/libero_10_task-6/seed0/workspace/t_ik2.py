from robot import *
r = Robot()
def tilt_quat(yaw_deg, pitch_deg):
    # hand down, then pitch about world y (positive tilts the approach so the hand leans toward -x / robot side)
    return (Rot.from_euler("y", pitch_deg, degrees=True) * Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
seed = np.array([0.146,-0.686,0.031,-2.831,0.023,2.145,0.158])
for pitch in [0, 10, 20]:
    for z in [0.62, 0.58, 0.555, 0.53]:
        q = r.ik([-0.196, 0.056, z], tilt_quat(0, pitch), seed=seed)
        print(f"rim grasp pitch={pitch} z={z}:", None if q is None else np.round(q,3))
for pitch in [0, 10, 20]:
    for z in [0.50, 0.46, 0.445]:
        q = r.ik([-0.057, 0.115, z], tilt_quat(0, pitch), seed=seed)
        print(f"pudding pitch={pitch} z={z}:", None if q is None else np.round(q,3))
for z in [0.62, 0.59]:
    q = r.ik([0.126, 0.056, z], tilt_quat(0, 0), seed=seed); print(f"plate place z={z}:", None if q is None else np.round(q,3))
for y in [0.17, 0.16]:
    for z in [0.48, 0.45]:
        q = r.ik([0.126, y, z], tilt_quat(0, 0), seed=seed); print(f"pudding place y={y} z={z}:", None if q is None else np.round(q,3))
