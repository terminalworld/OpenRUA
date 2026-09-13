from robot import *
r = Robot("placepud")
def tq(yaw, pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
Q = tq(0,0); T = np.array([0.126, 0.17])
q = r.goto([T[0], T[1], 0.66], Q, 5.0)
print("fingers", np.round(r.fingers(),4))
base_fz = r.wrench()[2]
for z in [0.55, 0.49, 0.465, 0.452]:
    q = r.goto([T[0], T[1], z], Q, 2.0, seed=q)
    w = r.wrench(); tp,_ = r.tcp()
    print(f"   z {z}: tcp {np.round(tp,4)} dz={tp[2]-z:+.4f} Fz={w[2]:.2f} dF={w[2]-base_fz:+.2f}")
    if abs(tp[2]-z) > 0.005 or (w[2]-base_fz) > 3: print("   touchdown"); break
r.gripper(0.04)
print("fingers", np.round(r.fingers(),4))
q = r.goto([T[0], T[1], 0.62], Q, 2.5, seed=q)
# park the arm up and away so cameras see the scene
q = r.goto([-0.05, 0.0, 0.80], Q, 4.0, seed=q)
print("wrench", np.round(r.wrench(),2))
