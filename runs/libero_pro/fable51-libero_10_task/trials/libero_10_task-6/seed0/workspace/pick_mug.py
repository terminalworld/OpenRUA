from robot import *
r = Robot("pick")
def tq(pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
MUG = np.array([-0.196, 0.016]); RIM_Z = 0.575
G = np.array([MUG[0], MUG[1] + 0.040])        # gripper centre over the +y wall
print("wrench before", np.round(r.wrench(),2))
print("1. pre-grasp high")
q1 = r.goto([G[0], G[1], 0.72], tq(10), 3.0)
print("   wrench", np.round(r.wrench(),2))
print("2. descend to rim height +1cm (tips just above rim)")
q2 = r.goto([G[0], G[1], 0.595], tq(10), 2.5, seed=q1)
print("   wrench", np.round(r.wrench(),2))
print("3. descend into rim")
q3 = r.goto([G[0], G[1], 0.555], tq(10), 2.0, seed=q2)
print("   wrench", np.round(r.wrench(),2))
np.save("q_grasp.npy", q3)
