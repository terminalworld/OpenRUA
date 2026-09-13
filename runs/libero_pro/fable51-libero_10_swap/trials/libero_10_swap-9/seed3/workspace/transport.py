import numpy as np, rclpy
from rob import Robot, quat_from_axes
from ins_geom import pose
def slerp(q0,q1,t):
    q0,q1=np.array(q0),np.array(q1)
    if np.dot(q0,q1)<0: q1=-q1
    d=np.clip(np.dot(q0,q1),-1,1); th=np.arccos(d)
    if th<1e-6: return q0
    return (np.sin((1-t)*th)*q0+np.sin(t*th)*q1)/np.sin(th)
r=Robot()
qv=quat_from_axes((0,0,-1),(1,0,0))
q33,info=pose(33,-0.17); H=info['H']; H0=H+np.array([0,-0.10,0])
print("H",np.round(H,3),"H0",np.round(H0,3))
p=r.fk()[0]
# 1 lift, 2 over, 3 descend
r.cart_path([((p[0],p[1],1.36),qv), ((-0.05,-0.25,1.36),qv), ((-0.119,-0.48,1.36),qv), ((-0.119,-0.482,1.20),qv)])
print("fingers",r.fingers())
# 4 tilt to 33 deg while descending to H0
wps=[]
for t in (0.34,0.67,1.0):
    pos=np.array([-0.119,-0.482,1.20])*(1-t)+H0*t
    wps.append((pos,slerp(qv,q33,t)))
r.cart_path(wps, min_step_t=1.0)
print("hand",np.round(r.fk()[0],3),"quat",np.round(r.fk()[1],3),"target q33",np.round(q33,3),"fingers",r.fingers())
rclpy.shutdown()
