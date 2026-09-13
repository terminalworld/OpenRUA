from rob import *
from moveit_msgs.srv import GetPositionFK
r=Robot('insp3')
def links(q):
    req=GetPositionFK.Request(); req.fk_link_names=[f'panda_link{i}' for i in range(1,9)]+['panda_hand','panda_leftfinger','panda_rightfinger']
    req.robot_state.joint_state=r._seed(q)
    fut=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
    for n,ps in zip(req.fk_link_names,fut.result().pose_stamped):
        p=ps.pose.position; print(f'  {n}: ({p.x:.3f},{p.y:.3f},{p.z:.3f})')
print('grasp cfg tilt40'); links([-1.206,0.448,0.741,-2.532,-0.64,2.059,0.915])
print('pre cfg tilt40'); links([-1.22,-0.153,0.853,-2.575,-0.15,1.822,0.46])
