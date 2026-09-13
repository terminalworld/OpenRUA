from rob import *
from pick_place import q_tilt, go, cart_path
r=Robot('retreat')
QG=q_tilt(np.radians(35),np.radians(-45))
tcp=r.tcp_world()
# straight up first, then to pre-grasp
qs=cart_path(r, r.joints(), [(tcp[0],tcp[1],0.62),(-0.25,-0.149,0.61)], QG)
go(r, qs, 4.0)
print('fingers', r.fingers(), 'tcp', r.tcp_world().round(3))
