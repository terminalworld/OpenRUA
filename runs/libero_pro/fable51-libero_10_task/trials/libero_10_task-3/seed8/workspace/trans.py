from robot import *
from plan3 import check_path
import plan3
np.set_printoptions(suppress=True,precision=3)
r=Robot(); q0=r.arm_q()
plan=np.load('plan3.npy',allow_pickle=True).item()
qpp=plan['prepush'][0]; print('prepush q',qpp,'tcp',r.tcp(qpp)[0])
check_path(r,[q0,qpp],n_sub=25,label='view->prepush')
print('dq',np.round(qpp-q0,2))
