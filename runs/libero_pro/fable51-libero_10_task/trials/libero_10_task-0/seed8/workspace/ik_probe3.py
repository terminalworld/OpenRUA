import numpy as np, rclpy
from rb import RB, M, ARM, TCP, q_down_yaw
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
r = RB("probe3")
p, q, tcp = r.hand_pose()
def call(label, xyz, quat, frame):
    req = GetPositionIK.Request()
    req.ik_request.group_name = M["planning"]["group"]
    req.ik_request.pose_stamped.header.frame_id = frame
    pp = req.ik_request.pose_stamped.pose
    pp.position.x, pp.position.y, pp.position.z = map(float, xyz)
    pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, quat)
    s = JointState(); cur = r.joints()
    for j in ARM: s.name.append(j); s.position.append(float(cur[j]))
    req.ik_request.robot_state.joint_state = s
    fut = r.ik.call_async(req)
    rclpy.spin_until_future_complete(r.n, fut, timeout_sec=60)
    res = fut.result()
    r.log(label, frame or "''", None if res is None else res.error_code.val,
          None if res is None or res.error_code.val != 1 else np.round(res.solution.joint_state.position[:7], 3))
call("hand world coords", p, q, "")
call("hand world coords", p, q, "world")
call("hand base coords", np.array(p)-r.base, q, "panda_link0")
for name, xyz in [("cc pregrasp", [0.108,-0.211,0.55]), ("tomato pregrasp", [-0.12,0.047,0.60]),
                  ("basket", [0.0,0.264,0.72]), ("park2", [-0.1,-0.4,0.70]), ("home-ish", [-0.05,0.0,0.68])]:
    hand = np.array(xyz) + [0,0,TCP]   # hand above tcp for downward hand
    call(name, hand - r.base, q_down_yaw(0), "panda_link0")
r.close()
