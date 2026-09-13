from lib import *
r = Robot("t6")
for link in ("panda_link8","panda_hand"):
    pos, quat = r.fk_world(link=link)
    R = Rot.from_quat(quat).as_matrix()
    print(link, np.round(pos,4), "y axis", np.round(R[:,1],3))
