from robo import *
r = Robot()
BASKET = np.array([-0.013, 0.252])
tcp = r.tcp_world()[0]
print("gap", r.finger_gap())
print("raise"); r.move_to([tcp[0], tcp[1], 0.76], seconds=2.5)
print("over basket"); r.move_to([*BASKET, 0.76], seconds=4)
print("gap", r.finger_gap())
print("lower"); r.move_to([*BASKET, 0.63], seconds=2.5)
print("gap", r.finger_gap(), "wrench", r.wrench())
print("release"); r.gripper(0.04)
print("retreat"); r.move_to([*BASKET, 0.80], seconds=2.5)
print("DONE")
