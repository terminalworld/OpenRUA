import sys, numpy as np
from ctl import Ctl, TCP, M
from scipy.spatial.transform import Rotation as Rot
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
import rclpy
c = Ctl()
lim = np.array(M["actuators"][0]["limits_rad"])
def ik(hp, q, seed, t=1.0):
    req = GetPositionIK.Request(); req.ik_request.group_name = M["planning"]["group"]
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = map(float, hp)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
    s = JointState(); s.name = list(M["actuators"][0]["joints"]); s.position = [float(v) for v in seed]
    req.ik_request.robot_state.joint_state = s
    req.ik_request.timeout.sec = int(t); req.ik_request.timeout.nanosec = int((t % 1) * 1e9)
    fut = c.ik.call_async(req); rclpy.spin_until_future_complete(c.node, fut, timeout_sec=30)
    r = fut.result()
    if r is None or r.error_code.val != 1: return None
    sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
    return [sol[j] for j in s.name]
v = list(map(float, sys.argv[1].split(",")))
n = int(sys.argv[2]) if len(sys.argv) > 2 else 20
if len(v) == 5:
    x, y, z, yaw, pitch = v
    R = Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_euler("x", 180, degrees=True)
else:
    tcp = np.array(v[0:3]); f = np.array(v[3:6]); d = np.array(v[6:9])
    d = d / np.linalg.norm(d); f = f - d * (f @ d); f = f / np.linalg.norm(f)
    R = Rot.from_matrix(np.column_stack([np.cross(f, d), f, d])); x, y, z = tcp
q = (R * Rot.from_euler("z", 45, degrees=True)).as_quat(); hz = R.as_matrix()[:, 2]; hp = np.array([x, y, z]) - TCP * hz
rng = np.random.default_rng(0); sols = []
seeds = [c.arm_q()] + [rng.uniform(lim[:, 0], lim[:, 1]) for _ in range(n)]
for sd in seeds:
    s = ik(hp, q, sd)
    if s is not None and not any(np.abs(np.array(s) - np.array(o)).max() < 0.05 for o in sols):
        sols.append(s); print("SOL", np.round(s, 2).tolist(), flush=True)
print("n_sols", len(sols))
