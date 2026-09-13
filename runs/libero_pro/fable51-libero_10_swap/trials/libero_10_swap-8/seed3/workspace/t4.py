from rob import *
r = Robot()
q0 = r.arm_q()
R = side_grasp_R(0, 10)
TABLE=0.894
potA=np.array([-0.199,-0.200]); d=0.01
seeds=[q0,[0,-0.785,0,-2.356,0,1.571,0.785],[-0.4,0.3,0,-2.2,0,2.5,0.4],[-0.3,0.6,0.1,-1.8,0,2.4,0.5],[0.0,0.0,0.0,-1.5,0.0,1.5,0.8]]
P1=np.array([potA[0]-d-0.08, potA[1], TABLE+0.15])
sol=r.best_ik_tcp(P1,R,seeds)
print("P1 best", np.round(sol,3))
P2=np.array([potA[0]-d-0.08, potA[1], TABLE+0.04])
sol2=r.best_ik_tcp(P2,R,[sol]+seeds)
print("P2 best", np.round(sol2,3))
P3=np.array([potA[0]-d, potA[1], TABLE+0.04])
sol3=r.best_ik_tcp(P3,R,[sol2]+seeds)
print("P3 best", np.round(sol3,3))
