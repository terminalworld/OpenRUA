"""FK via MoveIt: python3 fk.py j1,...,j7  (or 'cur' for current) -> hand pose in planning frame + world"""
import sys, rclpy, yaml
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
M = yaml.safe_load(open("/workspace/machine.yaml"))
JOINTS = next(a for a in M["actuators"] if a["kind"]=="joint_trajectory")["joints"]
def main():
    rclpy.init(); node = rclpy.create_node("fk")
    if sys.argv[1] == "cur":
        js = {}
        node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
        while "m" not in js: rclpy.spin_once(node, timeout_sec=0.2)
        d = dict(zip(js["m"].name, js["m"].position)); q = [d[j] for j in JOINTS]
        print("fingers:", d["panda_finger_joint1"], d["panda_finger_joint2"])
    else:
        q = [float(x) for x in sys.argv[1].split(",")]
    cli = node.create_client(GetPositionFK, "/compute_fk"); cli.wait_for_service(10)
    req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
    req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = q
    fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
    r = fut.result(); p = r.pose_stamped[0].pose
    print("joints:", [round(x,4) for x in q])
    print(f"hand (base frame): x={p.position.x:.4f} y={p.position.y:.4f} z={p.position.z:.4f} q=({p.orientation.x:.4f},{p.orientation.y:.4f},{p.orientation.z:.4f},{p.orientation.w:.4f})")
    print(f"hand (world): x={p.position.x-0.75:.4f} y={p.position.y:.4f} z={p.position.z+0.912:.4f}")
    rclpy.shutdown()
main()
