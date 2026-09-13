from robot import *
r = Robot("placemug")
def tq(yaw, pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
Q = tq(0,0)
PLATE = np.array([0.126, 0.016]); G = PLATE + [0, 0.040]
q = r.goto([G[0], G[1], 0.72], Q, 4.0)
print("fingers", np.round(r.fingers(),4), "wrench", np.round(r.wrench(),2))
base_fz = r.wrench()[2]
for z in [0.64, 0.60, 0.585, 0.575, 0.565]:
    q = r.goto([G[0], G[1], z], Q, 1.5, seed=q)
    w = r.wrench(); tp,_ = r.tcp()
    print(f"   z {z}: tcp {np.round(tp,4)} dz={tp[2]-z:+.4f} Fz={w[2]:.2f} dF={w[2]-base_fz:+.2f}")
    if abs(tp[2]-z) > 0.005 or (w[2]-base_fz) > 3:
        print("   touchdown"); break
r.gripper(0.04)
print("fingers", np.round(r.fingers(),4))
q = r.goto([G[0], G[1], 0.70], Q, 2.5, seed=q)
print("wrench", np.round(r.wrench(),2))
