import sys, rclpy, yaml
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
M = yaml.safe_load(open("/workspace/machine.yaml"))
arm = M["actuators"][0]["joints"]
rclpy.init(); node = rclpy.create_node("fk")
js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
while "m" not in js: rclpy.spin_once(node, timeout_sec=0.2)
cli = node.create_client(GetPositionFK, "/compute_fk"); cli.wait_for_service()
req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
seed = JointState()
if len(sys.argv) > 1:
    seed.name = arm; seed.position = [float(x) for x in sys.argv[1].split(",")]
else:
    for n,p in zip(js["m"].name, js["m"].position):
        if n in arm: seed.name.append(n); seed.position.append(p)
req.robot_state.joint_state = seed
fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
r = fut.result()
p = r.pose_stamped[0].pose
print("frame", r.pose_stamped[0].header.frame_id, "err", r.error_code.val)
print(f"hand(base) pos=({p.position.x:.4f},{p.position.y:.4f},{p.position.z:.4f}) q=({p.orientation.x:.4f},{p.orientation.y:.4f},{p.orientation.z:.4f},{p.orientation.w:.4f})")
print(f"hand(world) pos=({p.position.x-0.66:.4f},{p.position.y:.4f},{p.position.z+0.912:.4f})")
print("joints", dict(zip(seed.name, [round(x,4) for x in seed.position])))
