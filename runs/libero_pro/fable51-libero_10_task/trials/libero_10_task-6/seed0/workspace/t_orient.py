from robot import *
r = Robot("orient")
def tq(yaw, pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
pos, quat = r.fk()
print("current hand pos", np.round(pos,4), "quat", np.round(quat,4))
print("requested quat", np.round(tq(90,0),4))
R = Rot.from_quat(quat).as_matrix()
print("hand axes in world: x=",R[:,0].round(3)," y=",R[:,1].round(3)," z=",R[:,2].round(3))
Rr = Rot.from_quat(tq(90,0)).as_matrix()
print("requested axes:     x=",Rr[:,0].round(3)," y=",Rr[:,1].round(3)," z=",Rr[:,2].round(3))
print("angle between:", np.degrees((Rot.from_quat(quat).inv()*Rot.from_quat(tq(90,0))).magnitude()).round(2))
print("q", np.round(r.q(),4))
# also check link8 vs hand: FK for panda_link8
from moveit_msgs.srv import GetPositionFK
req = GetPositionFK.Request(); req.fk_link_names=["panda_link8","panda_hand","panda_leftfinger","panda_rightfinger"]
req.robot_state.joint_state.name=list(ARM); req.robot_state.joint_state.position=[float(v) for v in r.q()]
fut=r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
for n,ps in zip(fut.result().fk_link_names, fut.result().pose_stamped):
    p=ps.pose.position; o=ps.pose.orientation
    print(n, np.round([p.x,p.y,p.z],4), np.round([o.x,o.y,o.z,o.w],4))
