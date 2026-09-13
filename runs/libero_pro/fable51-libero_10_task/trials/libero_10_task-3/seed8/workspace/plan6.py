from robot import *
from plan3 import line, check_path
import plan4
np.set_printoptions(suppress=True,precision=3)
r=Robot()
p4=np.load('plan4.npy',allow_pickle=True).item()
qmid=p4['mid4'][-1]; MID=r.tcp(qmid)[0]; R_D=plan4.R_D
PREDROP=np.array([0.0,0.15,1.20]); DROP=np.array([0.0,0.15,1.11])
pd=line(r,MID,PREDROP,R_D,qmid,4); dr=line(r,PREDROP,DROP,R_D,pd[-1],3); up=line(r,DROP,PREDROP,R_D,dr[-1],2)
print('predrop',pd[-1],'drop',dr[-1])
# custom bottle points for centre grasp: +-0.06 along x_h, radius 0.02
def bpts(tcp,R):
    pts=[]
    for s in np.linspace(-0.06,0.06,5):
        for ang in np.linspace(0,2*np.pi,6,endpoint=False):
            pts.append(tcp+s*R[:,0]+0.02*(np.cos(ang)*R[:,1]+np.sin(ang)*R[:,2]))
    return np.array(pts)
import plan3; plan3.bottle_pts=bpts
qs=[p4['lift4'][-1],qmid]+pd+dr
check_path(r,qs,held=True,n_sub=8,label='lift->mid->drop')
np.save('plan6.npy',{'predrop6':pd,'drop6':dr,'up6':up},allow_pickle=True)
