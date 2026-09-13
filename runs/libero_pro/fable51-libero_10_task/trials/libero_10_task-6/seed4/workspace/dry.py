from robot import *
r = Robot()
q0,_ = r.joints()
TOP = np.array([1.0, 0, 0, 0])
MUG = np.array([-0.188, 0.033]); RIM_Z = 0.575; R_MUG = 0.044
grasp_xy = MUG + [0, R_MUG]
PLATE = np.array([0.145, 0.013])
PUD = np.array([-0.021, 0.103]); pud_quat = quat_mul(quat_z(math.radians(-14)), TOP)
tests = {
 "mug pre":   ([*grasp_xy, 0.65], TOP),
 "mug grasp": ([*grasp_xy, 0.545], TOP),
 "mug lift":  ([*grasp_xy, 0.72], TOP),
 "plate above": ([*(PLATE+[0,R_MUG]), 0.72], TOP),
 "plate place": ([*(PLATE+[0,R_MUG]), 0.585], TOP),
 "pud pre":   ([*PUD, 0.56], pud_quat),
 "pud grasp": ([*PUD, 0.445], pud_quat),
 "pud dest above": ([PLATE[0], 0.21, 0.60], pud_quat),
 "pud dest place": ([PLATE[0], 0.21, 0.45], pud_quat),
 "pud dest place2": ([PLATE[0], 0.20, 0.45], TOP),
}
seed = q0
for k,(p,qt) in tests.items():
    s = r.ik_tcp(p, qt, seed=seed)
    print(f"{k:16s} tcp={np.round(p,3)} -> {'OK ' + str(np.round(s,2)) if s is not None else 'FAIL'}")
    if s is not None: seed = s
