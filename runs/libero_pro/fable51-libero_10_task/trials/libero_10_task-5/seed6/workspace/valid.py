import sys, numpy as np, rclpy
from ctl import *
from moveit_msgs.srv import GetStateValidity
def check(r, q, c=None):
    c = c or r.node.create_client(GetStateValidity,'/check_state_validity'); c.wait_for_service(10)
    req=GetStateValidity.Request(); req.group_name='panda_arm'
    req.robot_state.joint_state.name=list(ARM)+["panda_finger_joint1","panda_finger_joint2"]; req.robot_state.joint_state.position=list(map(float,q))+[0.04,0.04]
    f=c.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=30); res=f.result()
    return res.valid, [(x.contact_body_1,x.contact_body_2,round(x.depth,4)) for x in res.contacts]
if __name__=="__main__":
    r=Robot('v'); q=[float(v) for v in sys.argv[1:8]]; print(check(r,q))
