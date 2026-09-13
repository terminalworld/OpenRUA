import sys, time, rclpy
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
rclpy.init(); n = rclpy.create_node("gtest")
clk = {}; js = {}
n.create_subscription(Clock, "/clock", lambda m: clk.__setitem__("t", m.clock.sec + m.clock.nanosec*1e-9), 10)
n.create_subscription(JointState, "/joint_states", lambda m: js.__setitem__("f", [p for nm,p in zip(m.name,m.position) if "finger" in nm]), 10)
c = ActionClient(n, GripperCommand, "/franka_gripper/gripper_action"); c.wait_for_server(10)
for _ in range(10): rclpy.spin_once(n, timeout_sec=0.1)
print("clock before", clk.get("t"), "fingers", js.get("f"))
g = GripperCommand.Goal(); g.command.position = float(sys.argv[1]); g.command.max_effort = 30.0
fb = []
f = c.send_goal_async(g, feedback_callback=lambda m: fb.append((m.feedback.position, m.feedback.reached_goal, m.feedback.stalled)))
rclpy.spin_until_future_complete(n, f, timeout_sec=30); h = f.result()
t0=time.time(); rf = h.get_result_async(); rclpy.spin_until_future_complete(n, rf, timeout_sec=120)
r = rf.result(); print("status", r.status, "result pos", r.result.position, "effort", r.result.effort, "reached", r.result.reached_goal, "stalled", r.result.stalled, "wall", round(time.time()-t0,2))
print("feedback n", len(fb), fb[:3], fb[-3:])
for _ in range(10): rclpy.spin_once(n, timeout_sec=0.1)
print("clock after", clk.get("t"), "fingers", js.get("f"))
