from ctl import *
c = Ctl()
print(np.round(yaw_down(0),4), quat_R(*yaw_down(np.pi/2))[:,1].round(3))
print("re-orient above mug"); c.move([-0.2255,0.035,0.70], DOWN, secs=3.0)
p,q = c.hand_world(); R=quat_R(*q); print("hand q", np.round(q,4), "finger axis", R[:,1].round(3), "approach", R[:,2].round(3))
