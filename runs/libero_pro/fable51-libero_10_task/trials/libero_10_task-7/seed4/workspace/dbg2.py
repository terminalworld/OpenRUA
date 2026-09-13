from rob import *
import time
from rosgraph_msgs.msg import Clock
r = Robot()
clk={}
r.node.create_subscription(Clock, "/clock", lambda m: clk.__setitem__("t", m.clock.sec+m.clock.nanosec*1e-9), 1)
sol = r.ik_world([-0.06, -0.146, 0.60], Q_DOWN_Y)
q0=np.array(r.arm_q()); print("delta", np.round(np.array(sol)-q0,3))
goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names=list(ARM)
pt = JointTrajectoryPoint(positions=[float(x) for x in sol]); pt.time_from_start = Duration(sec=6)
goal.trajectory.points=[pt]
r.spin(0.2); print("clock at start", clk.get("t"))
send = r.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(r.node, send, timeout_sec=60)
gh = send.result(); rf = gh.get_result_async()
t0=time.time(); last=-1
while not rf.done():
    r.spin(0.05)
    if time.time()-last > 1.0:
        last=time.time(); q=r.arm_q()
        print(f"wall={time.time()-t0:5.1f} sim={clk.get('t',0):8.3f} err={np.abs(np.array(q)-np.array(sol)).max():.3f}", flush=True)
res=rf.result().result; print("code", res.error_code, res.error_string, "sim", clk.get("t"))
q=r.arm_q(); print("final err", np.abs(np.array(q)-np.array(sol)).max().round(4), "tcp", r.tcp_world(q)[0].round(4))
