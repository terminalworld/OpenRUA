from robot import *
from plan3 import obstacles, bottle_pts
r=Robot()
for tilt in (15,25,35):
    c,s=np.cos(np.radians(tilt)),np.sin(np.radians(tilt))
    R=rot_from_axes([0,c,-s],[1,0,0])
    for x in (0.09,0.10,0.115):
        for y in (0.15,0.20):
            tcp=np.array([x,y,0.965])
            q=best_ik(r,tcp,R,n_rand=6)
            if q is None: print(tilt,x,y,'FAIL'); continue
            p,Rh=fk_links(r,q)['panda_hand']
            hp=hand_points(p,Rh,0.0)
            print(tilt,x,y,np.round(q,2),'hits',obstacles(hp),'clr',min(c for _,c,_ in link_clearance(r,q)))
