"""IK via MoveIt: python3 ik.py x y z qx qy qz qw [seed_j1,...,j7|cur] -> joint solution (no motion)"""
import sys, rclpy, yaml
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
M = yaml.safe_load(open("/workspace/machine.yaml"))
JOINTS = next(a for a in M["actuators"] if a["kind"]=="joint_trajectory")["joints"]
def solve(node, cli, pose, seed, timeout=60):
    req = GetPositionIK.Request()
    req.ik_request.group_name = "panda_arm"
    req.ik_request.pose_stamped.header.frame_id = ""
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = pose[:3]
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = pose[3:]
    req.ik_request.robot_state.joint_state.name = JOINTS
    req.ik_request.robot_state.joint_state.position = list(seed)
    req.ik_request.avoid_collisions = False
    fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    r = fut.result()
    if r is None: return None, "timeout"
    if r.error_code.val != 1: return None, r.error_code.val
    d = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
    return [d[j] for j in JOINTS], 1
def current(node):
    js = {}
    sub = node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
    while "m" not in js: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    d = dict(zip(js["m"].name, js["m"].position)); return [d[j] for j in JOINTS]
if __name__ == "__main__":
    rclpy.init(); node = rclpy.create_node("ik")
    cli = node.create_client(GetPositionIK, "/compute_ik"); cli.wait_for_service(10)
    pose = [float(x) for x in sys.argv[1:8]]
    seed = current(node) if len(sys.argv) < 9 or sys.argv[8]=="cur" else [float(x) for x in sys.argv[8].split(",")]
    sol, code = solve(node, cli, pose, seed)
    print("code", code, "sol", None if sol is None else ",".join(f"{x:.4f}" for x in sol))
    rclpy.shutdown()
