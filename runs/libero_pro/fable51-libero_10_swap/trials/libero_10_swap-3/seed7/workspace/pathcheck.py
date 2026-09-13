import sys, numpy as np, rclpy
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
J=[f"panda_joint{i}" for i in range(1,8)]
cur=np.array([-1.6696,-0.4327,1.7811,-2.4396,-0.5233,2.3642,-1.2694])
tgt=np.array([float(v) for v in sys.argv[1].split(",")])
rclpy.init(); node=rclpy.create_node("pc")
cli=node.create_client(GetPositionFK,"/compute_fk"); cli.wait_for_service()
links=["panda_hand","panda_link7","panda_link6","panda_link5","panda_link4"]
for a in np.linspace(0,1,11):
    q=cur+(tgt-cur)*a
    req=GetPositionFK.Request(); req.fk_link_names=links
    req.robot_state.joint_state=JointState(name=J, position=list(q))
    f=cli.call_async(req); rclpy.spin_until_future_complete(node,f)
    out=[]
    for ps in f.result().pose_stamped:
        p=ps.pose.position; out.append(f"{ps.header.frame_id[6:]}=({p.x:.2f},{p.y:.2f},{p.z:.2f})")
    # TCP
    ps=f.result().pose_stamped[0]; o=ps.pose.orientation
    x,y,z,w=o.x,o.y,o.z,o.w
    zax=np.array([2*(x*z+y*w),2*(y*z-x*w),1-2*(x*x+y*y)])
    tcp=np.array([ps.pose.position.x,ps.pose.position.y,ps.pose.position.z])+0.1034*zax
    print(f"a={a:.1f} tcp=({tcp[0]:.2f},{tcp[1]:.2f},{tcp[2]:.2f}) "+" ".join(out))
