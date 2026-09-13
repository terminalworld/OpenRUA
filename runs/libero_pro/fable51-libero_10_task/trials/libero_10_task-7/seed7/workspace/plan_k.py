from robot import *
import pickle
r = Robot()
log("current q", r.arm_q().round(3))
for yaw in (90, -90):
    Q = Robot.quat_topdown(yaw)
    chain = r.plan_descent([-0.224, -0.126, 0.475], Q, 0.64, n_seeds=40, margin=0.10)
    if chain is not None:
        log(f"yaw {yaw}: top q {chain[0].round(3)} grasp q {chain[-1].round(3)} len {len(chain)}")
        pickle.dump((yaw, chain), open(f"chain_k_{yaw}.pkl", "wb"))
