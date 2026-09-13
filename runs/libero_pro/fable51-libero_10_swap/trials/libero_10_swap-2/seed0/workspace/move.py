#!/usr/bin/env python3
"""move.py x y z yaw_deg [--sec S] [--hand] [--dry] [--j7 VAL]
Top-down hand (hand z pointing down) at world TCP position (x,y,z) with hand
x-axis yawed yaw_deg about world z (yaw 0 => fingers span world y).
Uses IK (tip = panda_link8, 45deg offset handled), FK-verifies, sends FJT.
--hand: (x,y,z) is the hand-frame origin instead of the TCP.
"""
import sys, math, time, subprocess
import numpy as np, rclpy, yaml
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = M["actuators"][0]["joints"]
TCP = M["hand"]["tcp_offset_m"]

def parse():
    a = sys.argv[1:]
    opts = {"sec": 4.0, "hand": False, "dry": False, "j7": None, "tilt": 0.0}
    pos = []
    i = 0
    while i < len(a):
        if a[i] == "--sec": opts["sec"] = float(a[i+1]); i += 2
        elif a[i] == "--j7": opts["j7"] = float(a[i+1]); i += 2
        elif a[i] == "--tilt": opts["tilt"] = float(a[i+1]); i += 2
        elif a[i] == "--hand": opts["hand"] = True; i += 1
        elif a[i] == "--dry": opts["dry"] = True; i += 1
        else: pos.append(float(a[i])); i += 1
    return pos, opts

def main():
    pos, o = parse()
    x, y, z, yaw = pos
    R_hand = Rot.from_euler("x", o["tilt"], degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_euler("x", 180, degrees=True)
    tcp_goal = np.array([x, y, z])
    if not o["hand"]:
        x, y, z = tcp_goal - R_hand.apply([0, 0, TCP])
    else:
        tcp_goal = tcp_goal + R_hand.apply([0, 0, TCP])
    R_l8 = R_hand * Rot.from_euler("z", 45, degrees=True)
    q = R_l8.as_quat()  # x,y,z,w
    rclpy.init(); node = rclpy.create_node("mv")
    js = {}
    node.create_subscription(JointState, "/joint_states", lambda m: js.__setitem__("m", m), 1)
    while "m" not in js: rclpy.spin_once(node, timeout_sec=0.2)
    cur = {n: p for n, p in zip(js["m"].name, js["m"].position)}
    ik = node.create_client(GetPositionIK, "/compute_ik"); ik.wait_for_service()
    fk = node.create_client(GetPositionFK, "/compute_fk"); fk.wait_for_service()
    req = GetPositionIK.Request()
    req.ik_request.group_name = "panda_arm"
    req.ik_request.pose_stamped.header.frame_id = ""
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    seed = JointState(); seed.name = list(ARM); seed.position = [cur[j] for j in ARM]
    req.ik_request.robot_state.joint_state = seed
    req.ik_request.avoid_collisions = False
    req.ik_request.timeout.sec = 3
    fut = ik.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
    r = fut.result()
    if r is None or r.error_code.val != 1:
        raise SystemExit(f"IK failed: {None if r is None else r.error_code.val}")
    sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
    target = [sol[j] for j in ARM]
    if o["j7"] is not None: target[6] = o["j7"]
    # FK verify
    fr = GetPositionFK.Request(); fr.fk_link_names = ["panda_hand"]
    fr.robot_state.joint_state.name = list(ARM); fr.robot_state.joint_state.position = target
    fut = fk.call_async(fr); rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
    hp = fut.result().pose_stamped[0].pose
    hq = Rot.from_quat([hp.orientation.x, hp.orientation.y, hp.orientation.z, hp.orientation.w])
    tcp = np.array([hp.position.x, hp.position.y, hp.position.z]) + hq.apply([0, 0, TCP])
    ang = (hq.inv() * R_hand).magnitude() * 180 / math.pi
    perr = np.linalg.norm(tcp - tcp_goal)
    print("target joints:", ",".join(f"{v:.4f}" for v in target))
    print(f"FK check: tcp=({tcp[0]:.4f},{tcp[1]:.4f},{tcp[2]:.4f}) pos_err={perr*1000:.1f}mm orient_err={ang:.1f}deg")
    dj = np.abs(np.array(target) - np.array(seed.position)); print("max joint delta: %.2f rad" % dj.max())
    if perr > 0.005 or ang > 3:
        raise SystemExit("FK mismatch; not moving")
    if o["dry"]: return
    rclpy.shutdown()
    rc = subprocess.run([sys.executable, "/workspace/tools/action/fjt_send.py",
                         ",".join(f"{v:.6f}" for v in target), str(o["sec"])])
    subprocess.run([sys.executable, "/workspace/fk.py"])

if __name__ == "__main__":
    main()
