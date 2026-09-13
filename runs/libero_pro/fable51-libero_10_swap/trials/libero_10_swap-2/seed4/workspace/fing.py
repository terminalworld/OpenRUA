from moka_plan import *
m = Mover("fing")
for i in range(3):
    print("fingers", m.fingers(), "wrench", np.round(wrench(m,5),2))
