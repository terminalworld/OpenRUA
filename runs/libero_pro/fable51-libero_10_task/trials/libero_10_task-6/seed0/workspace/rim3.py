from robot import *
r = Robot("rim3")
def tq(yaw, pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
def axes():
    pos, quat = r.fk(); R = Rot.from_quat(quat).as_matrix(); return R[:,1].round(3), R[:,2].round(3)
MUG = np.array([-0.195, 0.016]); G = np.array([MUG[0], MUG[1] + 0.040])
Q = tq(0, 0)
print("finger axis now", axes())
q = r.goto([G[0], G[1], 0.62], Q, 4.0)
print("   finger axis, approach:", axes(), " fingers", np.round(r.fingers(),4))
base_fz = r.wrench()[2]
for z in [0.59, 0.575, 0.565, 0.555, 0.545]:
    q = r.goto([G[0], G[1], z], Q, 1.5, seed=q)
    w = r.wrench(); tp,_ = r.tcp()
    print(f"   z target {z}: tcp {np.round(tp,4)} dz={tp[2]-z:+.4f}  Fz={w[2]:.2f} (dF={w[2]-base_fz:+.2f}) axis {axes()[0]}")
    if abs(tp[2]-z) > 0.006 or abs(w[2]-base_fz) > 4:
        print("   BLOCKED"); break
np.save("q_rim.npy", q)
