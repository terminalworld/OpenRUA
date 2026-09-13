from robot import *
r = Robot("liftmug")
def tq(yaw, pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
Q = tq(0,0); G = [-0.195, 0.056]
q = r.goto([G[0], G[1], 0.60], Q, 2.0)
print("fingers", np.round(r.fingers(),4), "wrench", np.round(r.wrench(),2))
q = r.goto([G[0], G[1], 0.72], Q, 2.5, seed=q)
print("fingers", np.round(r.fingers(),4), "wrench", np.round(r.wrench(),2))
