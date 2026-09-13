from rob import *
r=Robot('insp')
q=r.joints(); print('cur joints', np.round(q,3)); print('tcp', r.tcp_world().round(3), 'fingers', r.fingers())
x,y=-0.25,-0.149
for label,quat in [('yaw0',Q_DOWN),('yaw90',q_yaw_down(np.pi/2)),('yaw-45',q_yaw_down(-np.pi/4)),('yaw45',q_yaw_down(np.pi/4))]:
    print('---',label)
    seed=q
    for z in [0.61,0.56,0.51,0.461]:
        s=r.ik_tcp_world((x,y,z), quat=quat, seed=seed)
        print(z, None if s is None else np.round(s,3))
        if s is not None: seed=s
