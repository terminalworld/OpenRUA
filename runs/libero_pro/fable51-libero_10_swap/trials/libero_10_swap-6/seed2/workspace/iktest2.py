import numpy as np, sys, rclpy
sys.path.insert(0,"/workspace")
from arm import Arm, down_quat, JOINTS, M
from scipy.spatial.transform import Rotation as Rot
from moveit_msgs.srv import GetStateValidity
a = Arm()
sv = a.node.create_client(GetStateValidity, "/check_state_validity"); sv.wait_for_service(10)
def valid(q):
    req = GetStateValidity.Request(); req.group_name = M["planning"]["group"]
    req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = list(map(float,q))
    fut = sv.call_async(req); rclpy.spin_until_future_complete(a.node, fut, timeout_sec=30)
    r = fut.result(); return r.valid, [(c.contact_body_1, c.contact_body_2) for c in r.contacts]
def pitched(pitch_deg, yaw_deg=0):
    return (Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_euler("y", pitch_deg, degrees=True) * Rot.from_euler("x", 180, degrees=True)).as_quat()
q0 = a.q(); home = np.array([0,-0.161,0,-2.445,0,2.227,0.785])
print("current valid?", valid(q0))
for pitch in [0, 15, 25, 35]:
    for tgt in [(-0.222,0.022,0.62), (-0.222,0.022,0.52), (-0.222,0.022,0.445)]:
        sol = a.solve_ik(tgt, pitched(pitch), seed=home, attempts=2)
        if sol is None: print(f"pitch={pitch} {tgt}: IK none"); continue
        v = valid(sol)
        print(f"pitch={pitch} {tgt}: q={sol.round(2)} j4={sol[3]:.2f} valid={v}")
