from robot import *
r=Robot()
q0=r.arm_q()
# reachability of straight-down grasp (fingers along y) at TCP z=0.94 for various x, y=0.03
for pitch in [0.0, 0.3, 0.5]:
  for x in [0.10,0.14,0.17,0.20,0.23]:
    R=grasp_R(yaw=0, pitch=pitch)
    s=r.ik_tcp([x,0.03,0.94],R,seed=q0,timeout=2)
    print(f'pitch={pitch} x={x}: {"OK "+str(s.round(2)) if s is not None else "fail"}')
# reachability at pot positions
for name,(x,y) in {'pot1':(-0.193,-0.200),'pot2':(-0.040,0.253)}.items():
    for yaw in [0, np.pi/2]:
        s=r.ik_tcp([x,y,0.94],grasp_R(yaw=yaw),seed=q0,timeout=2)
        print(name,'yaw',yaw, None if s is None else s.round(2))
