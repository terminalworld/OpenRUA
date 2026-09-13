from ctl import *
import ctl; ctl.BASE_W = np.zeros(3)
c = Ctl()
print("move above mug"); c.move([-0.2255,0.035,0.70], DOWN, secs=3.0)
p,q = c.hand_world(); R=quat_R(*q); print("hand q", np.round(q,4), "finger axis (hand y) in world", R[:,1].round(3), "approach", R[:,2].round(3))
print("joints", np.round(c.arm_q(),3))
