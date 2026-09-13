from robo import *
r = Robot()
SOUP = np.array([-0.2387, -0.1451])
HOVER, GRASP = 0.62, 0.455

print("open gripper"); r.gripper(0.04)
print("hover above soup"); r.move_to([*SOUP, HOVER], seconds=4)
print("descend");
for z in (0.56, 0.50, GRASP):
    r.move_to([*SOUP, z], seconds=1.5)
print("wrench before", r.wrench())
print("close"); gap = r.gripper(0.0)
print("wrench after", r.wrench())
print("GRASP_GAP", gap)
if gap > 0.03:
    print("lift"); r.move_to([*SOUP, 0.70], seconds=3)
    print("gap after lift", r.finger_gap())
print("DONE")
