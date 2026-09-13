#!/usr/bin/env python3
"""Ask IK for the current FK pose, in world coords vs base coords; see which returns the current joints."""
import rclpy
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

JOINTS = ["panda_joint1","panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7"]
CUR = [0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]

rclpy.init(); node = rclpy.create_node("iktest")
cli = node.create_client(GetPositionIK, "/compute_ik"); cli.wait_for_service(10)

def ik(x,y,z,qx,qy,qz,qw):
    req = GetPositionIK.Request()
    req.ik_request.group_name = "panda_arm"
    req.ik_request.pose_stamped.header.frame_id = ""
    p = req.ik_request.pose_stamped.pose
    p.position.x,p.position.y,p.position.z = x,y,z
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w = qx,qy,qz,qw
    req.ik_request.robot_state.joint_state = JointState(name=JOINTS, position=CUR)
    req.ik_request.avoid_collisions = False
    fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
    r = fut.result()
    if r is None: return "timeout"
    if r.error_code.val != 1: return f"err {r.error_code.val}"
    sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
    return [round(sol[j],3) for j in JOINTS]

q = (0.9995966352021468, 0.0, -0.028400121347386207, 0.0)
print("world-frame pose :", ik(-0.05298564807835497, 0.0, 0.7776238083193787, *q))
print("base-frame pose  :", ik(-0.05298564807835497+0.51, 0.0, 0.7776238083193787-0.42, *q))
rclpy.shutdown()
