#!/usr/bin/env python3
"""IK feasibility probe: python3 ikprobe.py x y z qx qy qz qw [x y z qx qy qz qw ...]  (world coords, hand frame). No motion."""
import sys, rclpy, numpy as np
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
ARM=[f"panda_joint{i}" for i in range(1,8)]
def main():
    vals=list(map(float,sys.argv[1:])); poses=[vals[i:i+7] for i in range(0,len(vals),7)]
    rclpy.init(); node=rclpy.create_node("ikprobe")
    js={}
    node.create_subscription(JointState,"/joint_states",lambda m: js.setdefault("m",m),1)
    while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
    d=dict(zip(js["m"].name,js["m"].position))
    seed=[d[a] for a in ARM]
    cli=node.create_client(GetPositionIK,"/compute_ik"); cli.wait_for_service(timeout_sec=10)
    for p in poses:
        req=GetPositionIK.Request(); r=req.ik_request
        r.group_name="panda_arm"; r.pose_stamped.header.frame_id=""
        r.pose_stamped.pose.position.x,r.pose_stamped.pose.position.y,r.pose_stamped.pose.position.z=p[:3]
        r.pose_stamped.pose.orientation.x,r.pose_stamped.pose.orientation.y,r.pose_stamped.pose.orientation.z,r.pose_stamped.pose.orientation.w=p[3:]
        r.robot_state.joint_state.name=ARM; r.robot_state.joint_state.position=seed
        r.avoid_collisions=False
        r.timeout.sec=2
        fut=cli.call_async(req); rclpy.spin_until_future_complete(node,fut,timeout_sec=60)
        res=fut.result()
        if res is None: print(p[:3],"NO ANSWER"); continue
        if res.error_code.val!=1: print(np.round(p[:3],3),"FAIL",res.error_code.val); continue
        sol=dict(zip(res.solution.joint_state.name,res.solution.joint_state.position))
        q=[sol[a] for a in ARM]
        print(np.round(p[:3],3),"OK",",".join(f"{v:.4f}" for v in q))
    rclpy.shutdown()
main()
