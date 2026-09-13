from rob import *
from scipy.spatial.transform import Rotation as Rot
r=Robot('insp2')
q=r.joints(); print('cur joints', np.round(q,3), 'tcp', r.tcp_world().round(3))
x,y=-0.25,-0.149
def q_tilt(theta, yaw=0.0):
    c,s=np.cos(theta),np.sin(theta)
    R=np.array([[c,0,-s],[0,-1,0],[-s,0,-c]])
    R=Rot.from_euler('z',yaw).as_matrix()@R
    return tuple(Rot.from_matrix(R).as_quat())  # xyzw
for deg in [20,30,40,50]:
    quat=q_tilt(np.radians(deg))
    print('--- tilt',deg)
    seed=q
    for z in [0.66,0.61,0.56,0.51,0.461]:
        s=r.ik_tcp_world((x,y,z), quat=quat, seed=seed)
        print(z, None if s is None else np.round(s,3), '' if s is None else r.tcp_world(s).round(3))
        if s is not None: seed=s
