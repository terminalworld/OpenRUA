from robot import *
r=Robot()
q0=r.arm_q()
for pitch in [-0.25,-0.4,-0.55]:
  for x in [0.14,0.17,0.20,0.23]:
    R=grasp_R(yaw=0, pitch=pitch)
    s=r.ik_tcp([x,0.03,0.94],R,seed=q0,timeout=2)
    print(f'pitch={pitch} x={x}: {"OK "+str(s.round(2)) if s is not None else "fail"}')
