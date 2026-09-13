from rob import *
r = R()
print("joints", np.round(r.joints(),3))
print("fk hand", r.fk())
print("tcp", r.tcp())
print("fingers", r.fingers())
# test IK for key poses
Q_DOWN_Y = (1.0, 0.0, 0.0, 0.0)          # fingers along world y
Q_DOWN_X = (0.7071068, 0.7071068, 0.0, 0.0)  # fingers along world x
for name,(x,y,z,q) in {"knob_pre":(-0.197,0.201,1.05,Q_DOWN_Y),
                       "knob":(-0.197,0.201,0.95,Q_DOWN_Y),
                       "pan_pre":(-0.066,-0.085,1.15,Q_DOWN_X),
                       "pan":(-0.066,-0.085,1.005,Q_DOWN_X),
                       "stove_place":(-0.044,0.383,1.04,Q_DOWN_X),
                       "stove_place_alt":(-0.224,0.203,1.04,Q_DOWN_Y)}.items():
    j = r.ik(x,y,z,*q)
    print(name, None if j is None else np.round(j,3))
