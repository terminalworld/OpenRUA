from pp import *
r=Robot()
for j6 in [1.571, 2.0, 2.4, 2.79, 3.2]:
    q=np.array([0,-0.785,0,-2.356,0,j6,0.785])
    p,R=r.fk(q,'panda_hand'); print('j6',j6,'hand',p.round(3),'z',R[:,2].round(2),'y',R[:,1].round(2))
for j6 in [0.35, 0.8]:
    q=np.array([0,-0.785,0,-2.356,0,j6,0.785])
    p,R=r.fk(q,'panda_hand'); print('j6',j6,'hand',p.round(3),'z',R[:,2].round(2),'y',R[:,1].round(2))
# vary j2, j4 with j6 fixed at 2.79
for j2,j4 in [(0.0,-2.0),(0.5,-1.8),(0.8,-1.5),(1.0,-1.2),(0.3,-2.4)]:
    q=np.array([0,j2,0,j4,0,2.79- ( (j2-(-0.785)) + (j4-(-2.356)) ) ,0.785])
    p,R=r.fk(q,'panda_hand'); print('j2',j2,'j4',j4,'j6',round(q[5],3),'hand',p.round(3),'z',R[:,2].round(2))
