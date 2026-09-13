from robot import *
r = Robot("look")
def tq(yaw, pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
q = r.goto([-0.195, 0.016, 0.66], tq(90,0), 2.0)
q = r.goto([-0.19, -0.05, 0.72], tq(90,0), 2.5, seed=q)
print("wrench", np.round(r.wrench(),2))
