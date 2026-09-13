from rob import *
import time
r = Robot()
q0 = r.arm_q(); print("q0", np.round(q0,3))
tcp0 = r.tcp_world(q0)[0]; print("tcp0", tcp0.round(4))
sol = r.ik_world([-0.197, -0.139, 0.62], Q_DOWN_X)
print("ik sol", np.round(sol,3), "tcp of sol", r.tcp_world(sol)[0].round(4))
# send and sample joint states during execution
goal = FollowJointTrajectory.Goal()
goal.trajectory.joint_names = list(ARM)
pt = JointTrajectoryPoint(positions=[float(x) for x in sol]); pt.time_from_start = Duration(sec=4)
goal.trajectory.points=[pt]
send = r.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(r.node, send, timeout_sec=60)
gh = send.result(); rf = gh.get_result_async()
t0=time.time()
while not rf.done():
    r.spin(0.1)
    if int((time.time()-t0)*2) % 4 == 0:
        q = r.arm_q(); print(f"t={time.time()-t0:.1f} err={np.abs(np.array(q)-np.array(sol)).max():.3f} q={np.round(q,2)}", flush=True)
res = rf.result().result
print("code", res.error_code, res.error_string)
for i in range(6):
    q = r.arm_q(); print(f"post {i} err={np.abs(np.array(q)-np.array(sol)).max():.4f} tcp={r.tcp_world(q)[0].round(4)}"); time.sleep(0.5)
