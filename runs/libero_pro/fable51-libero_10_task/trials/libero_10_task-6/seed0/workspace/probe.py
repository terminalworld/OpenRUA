from robot import *
r = Robot("probe")
def tq(yaw, pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
C = np.array([-0.195, 0.016])
Q = tq(90, 0)
base_fz = r.wrench()[2]
q = r.goto([C[0], C[1], 0.62], Q, 3.0)
print("   wrench", np.round(r.wrench(),2))
for z in [0.585, 0.57, 0.555, 0.54, 0.525]:
    q = r.goto([C[0], C[1], z], Q, 1.5, seed=q)
    w = r.wrench(); tp,_ = r.tcp()
    print(f"   z target {z}: tcp z {tp[2]:.4f} dz={tp[2]-z:+.4f}  Fz={w[2]:.2f} (dF={w[2]-base_fz:+.2f})")
    if abs(tp[2]-z) > 0.006 or abs(w[2]-base_fz) > 4:
        print("   BLOCKED"); break
np.save("q_probe.npy", q)
