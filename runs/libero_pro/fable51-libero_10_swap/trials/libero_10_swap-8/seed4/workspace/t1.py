from robot import *
r=Robot()
q=r.arm_q(); print('q',q.round(3))
for link in ['panda_hand','panda_link8','panda_hand_tcp','panda_leftfinger']:
    try:
        p,R=r.fk(q,link); print(link,'pos',p.round(4),'z-axis',R[:,2].round(3),'y-axis',R[:,1].round(3))
    except Exception as e: print(link,e)
# IK check: does IK of current hand pose return current q?
p,R=r.fk(q,'panda_hand')
s=r.ik(p,R); print('ik back',None if s is None else (s-q).round(3))
