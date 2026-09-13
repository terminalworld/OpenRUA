from rob import *
r=Robot(); R=side_grasp_R(0,10)
print("gap",round(r.finger_gap(),4),"tcp",r.tcp()[0].round(3))
for p in [[0.068,-0.091,1.10],[0.0,-0.13,1.10],[0.13,-0.05,1.10],[0.21,-0.019,1.10],[0.21,-0.019,0.972]]:
    for ac in [True, False]:
        req = GetPositionIK.Request(); req.ik_request.group_name="panda_arm"; req.ik_request.pose_stamped.header.frame_id=""
        h=hand_pose_from_tcp(p,R); R8=R@Rot.from_euler("z",45,degrees=True).as_matrix(); q=Rot.from_matrix(R8).as_quat()
        pp=req.ik_request.pose_stamped.pose; pp.position.x,pp.position.y,pp.position.z=map(float,h); pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=map(float,q)
        req.ik_request.robot_state.joint_state.name=ARM; req.ik_request.robot_state.joint_state.position=r.arm_q(); req.ik_request.avoid_collisions=ac; req.ik_request.timeout.sec=2
        res=r._call(r.ik,req); print(p,"avoid",ac,"code",res.error_code.val, None if res.error_code.val!=1 else np.round([res.solution.joint_state.position[res.solution.joint_state.name.index(j)] for j in ARM],2))
