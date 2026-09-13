import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot()
def pitched(th, fingers=(1,0,0), sign=1):
    z=np.array([0,sign*np.sin(np.radians(th)),-np.cos(np.radians(th))]); return z, quat_from_axes(z,fingers)
tests=[]
P=np.array([-0.125,-0.49,0.95])
for th in (45,55,35):
    z,q=pitched(th); H=P-0.093*z; tests.append((f"pinch{th}",H,q)); tests.append((f"pre{th}",H-0.08*z,q))
# after lift and rotation: hand pointing -y/down 45, pinch at P+(0,0,0.15)
z,q=pitched(45,sign=-1); Pl=P+np.array([0,0,0.15]); tests.append(("rot45 lifted",Pl-0.093*z,q))
# yawed 90: hand pointing +x/down 45, fingers along y, pinch at rim +y point of mug centred (-0.17,-0.45), rim z 1.005 on table, pinch 1cm below
z=np.array([0.707,0,-0.707]); q=quat_from_axes(z,(0,1,0)); Pt=np.array([-0.17,-0.40,0.995]); tests.append(("yawed set",Pt-0.093*z,q)); tests.append(("yawed high",Pt+np.array([0,0,0.12])-0.093*z,q))
# handle pinch 45 deg: bar at (-0.17, -0.45-0.084, 0.965)
z,q=pitched(45); Pb=np.array([-0.17,-0.534,0.965]); tests.append(("bar pinch",Pb-0.093*z,q)); tests.append(("bar lifted",Pb+np.array([0,0,0.06])-0.093*z,q)); tests.append(("bar inserted",np.array([-0.17,-0.335,1.02])-0.093*z,q))
seed=r.arm_q()
for name,p,q in tests:
    s=r.ik(p,q,seed=seed,avoid=True); s2=s or r.ik(p,q,seed=seed,avoid=False)
    print(f"{name:14s} {np.round(p,3)} avoid={'OK' if s else 'FAIL'} free={'OK' if s2 else 'FAIL'}")
rclpy.shutdown()
