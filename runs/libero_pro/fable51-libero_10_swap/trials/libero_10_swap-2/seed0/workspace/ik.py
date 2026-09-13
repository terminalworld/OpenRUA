"""ik.py x y z qx qy qz qw  -> prints joint solution (no motion). Pose in whatever frame IK uses."""
import sys, rclpy, yaml
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
M = yaml.safe_load(open("/workspace/machine.yaml"))
arm = M["actuators"][0]["joints"]
x,y,z,qx,qy,qz,qw = map(float, sys.argv[1:8])
rclpy.init(); node = rclpy.create_node("ik")
js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
while "m" not in js: rclpy.spin_once(node, timeout_sec=0.2)
cli = node.create_client(GetPositionIK, "/compute_ik"); cli.wait_for_service()
req = GetPositionIK.Request()
req.ik_request.group_name = "panda_arm"
req.ik_request.pose_stamped.header.frame_id = ""
p = req.ik_request.pose_stamped.pose
p.position.x, p.position.y, p.position.z = x,y,z
p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx,qy,qz,qw
seed = JointState()
if len(sys.argv) > 8:
    seed.name = arm; seed.position = [float(v) for v in sys.argv[8].split(",")]
else:
    for n,pp in zip(js["m"].name, js["m"].position):
        if n in arm: seed.name.append(n); seed.position.append(pp)
req.ik_request.robot_state.joint_state = seed
req.ik_request.avoid_collisions = False
req.ik_request.timeout.sec = 2
fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
r = fut.result()
print("err", r.error_code.val)
if r.error_code.val == 1:
    sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
    print(",".join(f"{sol[j]:.5f}" for j in arm))
