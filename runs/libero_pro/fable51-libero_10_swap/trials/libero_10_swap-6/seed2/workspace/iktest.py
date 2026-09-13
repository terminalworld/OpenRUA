import numpy as np, sys
sys.path.insert(0,"/workspace")
from arm import Arm, down_quat
a = Arm()
q0 = a.q(); print("q0", q0.round(3))
for tgt in [(-0.222,0.022,0.62), (-0.222,0.022,0.55), (-0.222,0.022,0.44), (-0.15,0.022,0.62), (-0.222,0.022,0.72)]:
    for seed_name, seed in [("current", q0), ("home", np.array([0,-0.161,0,-2.445,0,2.227,0.785]))]:
        sol = a.solve_ik(tgt, down_quat(0), seed=seed, attempts=1)
        print(tgt, seed_name, None if sol is None else sol.round(3), None if sol is None else f"dist={np.abs(sol-seed).max():.2f}")
