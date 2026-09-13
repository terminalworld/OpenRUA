import rclpy, numpy as np
from rob import Robot, ARM
from moveit_msgs.srv import GetStateValidity
r=Robot()
q=r.arm_q(); print("q",np.round(q,3))
for l in ["panda_hand","panda_leftfinger","panda_link7","panda_link6"]:
    print(l, np.round(r.fk(q,l)[0],3))
cli=r.node.create_client(GetStateValidity,"/check_state_validity"); cli.wait_for_service(5)
req=GetStateValidity.Request(); req.group_name="panda_arm"
req.robot_state.joint_state.name=list(ARM); req.robot_state.joint_state.position=[float(v) for v in q]
fut=cli.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
res=fut.result(); print("valid",res.valid)
for c in res.contacts: print(" contact",c.contact_body_1,c.contact_body_2,np.round([c.position.x,c.position.y,c.position.z],3), "depth",round(c.depth,4))
rclpy.shutdown()
