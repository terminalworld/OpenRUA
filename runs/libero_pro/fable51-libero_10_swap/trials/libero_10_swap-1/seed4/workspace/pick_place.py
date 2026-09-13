import sys
from arm import Arm
OBJ = {"butter": (-0.084, 0.045, 0.449), "cream_cheese": (-0.001, -0.274, 0.455)}
BASKET = (0.007, 0.258)
TABLE = 0.425
x, y, top = OBJ[sys.argv[1]]
grasp_z = TABLE + 0.011
a = Arm()
print("open gripper"); a.gripper(0.04)
print("above object"); a.move((x, y, 0.56), seconds=3)
print("descend"); a.move((x, y, grasp_z), seconds=2.5)
print("close"); f = a.gripper(0.0)
if f < 0.005: raise SystemExit("GRASP FAILED: fingers closed on air")
print("lift"); a.move((x, y, 0.70), seconds=2.5)
j = a.joints(); print(f"  fingers after lift: {j['panda_finger_joint1']:.4f}")
print("to basket"); a.move((BASKET[0], BASKET[1], 0.72), seconds=3)
j = a.joints(); print(f"  fingers over basket: {j['panda_finger_joint1']:.4f}")
print("release"); a.gripper(0.04)
print("retreat up"); a.move((BASKET[0], BASKET[1] - 0.1, 0.75), seconds=2)
print("DONE")
