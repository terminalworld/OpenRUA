import sys
from moka_plan import *
m = Mover("moka_d")
TH = np.radians(12.0)
DT = D*np.cos(TH) - np.array([0,0,1.0])*np.sin(TH)      # approach tilted 12 deg down
Q_TILT = hand_quat(DT, +CC)
step = sys.argv[1]; dry = len(sys.argv) > 2 and sys.argv[2] == "dry"
p0,q0 = m.fk_world(); print("hand now", np.round(p0,4), "q", np.round(m.arm_q(),3))
print("wrench before", np.round(wrench(m),2))
if step == "back":
    t = p0 - 0.04*D; move_interp(m, t, Q_GRASP, n=2, seconds=2.5, dry=dry)
elif step == "tilt":
    t = p0.copy(); t[2] = 0.950; move_interp(m, t, Q_TILT, n=4, seconds=3.0, dry=dry)
elif step == "adv":
    dist = float(sys.argv[3]) if len(sys.argv) > 3 else None
    horiz = 0.1034*np.cos(TH)
    goal = np.array([C[0], C[1], 0.950]) + (TIP_PAST - horiz)*np.array([D[0], D[1], 0.0])
    if dist is not None: goal = p0 + dist*D; goal[2] = 0.950
    print("goal", np.round(goal,4)); move_interp(m, goal, Q_TILT, n=3, seconds=2.5, dry=dry)
elif step == "close":
    m.gripper(0.0)
elif step == "open":
    m.gripper(0.04)
p1,q1 = m.fk_world(); print("hand after", np.round(p1,4), "q", np.round(m.arm_q(),3), "fingers", np.round(m.fingers(),4))
print("wrench after", np.round(wrench(m),2))
