import rclpy, numpy as np, time, sys
from arm import Arm, ARM
from moveit_msgs.srv import GetPositionFK
from px import q2R
a = Arm(); a.fk_cli.wait_for_service(10)
def fkj(js):
    req = GetPositionFK.Request(); req.fk_link_names=['panda_hand']
    req.robot_state.joint_state.name = ARM; req.robot_state.joint_state.position = [float(v) for v in js]
    t=time.time(); fut = a.fk_cli.call_async(req); rclpy.spin_until_future_complete(a.node, fut, timeout_sec=90)
    r = fut.result(); print('fk took', round(time.time()-t,1))
    if r is None: print('FK none'); return
    p = r.pose_stamped[0].pose
    R = q2R(p.orientation)
    print('pos', np.round([p.position.x,p.position.y,p.position.z],4), 'quat', np.round([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w],4), 'hand y axis', R[:,1].round(3), 'z', R[:,2].round(3))
#fkj([0.05846165151957628, -0.16131001035395545, -0.0579201015032298, -2.4446575328760303, -0.01162123465199098, 2.226770915891463, 0.00890601661941885])
#fkj([0,-0.161,0,-2.4446,0,2.2268,0.7854])
#sol = a.ik(-0.203, 0, 1.2696, 0.7071, 0.7071, 0, 0); print('ik fingers-along-x:', sol)
#if sol: fkj(sol)
