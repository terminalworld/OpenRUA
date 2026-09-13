import numpy as np
from rob import *
from goto import ik_checked
r = Robot("w")
R1 = np.load("snaps/R_side.npy")
q0 = r.arm_q()
q = ik_checked(r, [-0.201, 0.012, 1.04], R1, q0)
print("q0", np.round(q0,3)); print("qt", np.round(q,3))
goal = FollowJointTrajectory.Goal()
goal.trajectory.joint_names = list(JOINTS)
pt = JointTrajectoryPoint(positions=[float(x) for x in q]); pt.time_from_start = Duration(sec=4)
goal.trajectory.points = [pt]
send = r.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(r.node, send, timeout_sec=60)
gh = send.result(); rf = gh.get_result_async()
t0 = time.time()
while not rf.done():
    rclpy.spin_once(r.node, timeout_sec=0.05)
    if time.time()-t0 > 0.5:
        t0 = time.time()
        js = r.js
        print(f"{js.header.stamp.sec}.{js.header.stamp.nanosec//10**8} j2={r.arm_q()[1]:.3f} err={np.round(r.arm_q()-q,3)} F={np.round(r.force(),1)}", flush=True)
res = rf.result().result
print("code", res.error_code, res.error_string)
