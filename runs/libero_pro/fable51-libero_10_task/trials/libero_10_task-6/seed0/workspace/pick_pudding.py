from robot import *
r = Robot("pickpud")
def tq(yaw, pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
Q = tq(0,0); BOX = np.array([-0.054, 0.114])
print("fingers", np.round(r.fingers(),4))
q = r.goto([BOX[0], BOX[1], 0.62], Q, 5.0)
pos, quat = r.fk(); print("   finger axis", Rot.from_quat(quat).as_matrix()[:,1].round(3))
base_fz = r.wrench()[2]
for z in [0.52, 0.48, 0.446]:
    q = r.goto([BOX[0], BOX[1], z], Q, 2.0, seed=q)
    w = r.wrench(); tp,_ = r.tcp()
    print(f"   z {z}: tcp {np.round(tp,4)} dz={tp[2]-z:+.4f} Fz={w[2]:.2f} dF={w[2]-base_fz:+.2f}")
    if abs(tp[2]-z) > 0.006 or abs(w[2]-base_fz) > 4: print("   BLOCKED"); break
np.save("q_pud.npy", q)
