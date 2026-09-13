"""Add table/cabinet/bottle boxes to the MoveIt planning scene; add valid(q) helper."""
from robot import *
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
def box(id_, center, size):
    co = CollisionObject(); co.header.frame_id = "world"; co.id = id_
    sp = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(s) for s in size])
    p = Pose(); p.position.x, p.position.y, p.position.z = map(float, center); p.orientation.w = 1.0
    co.primitives.append(sp); co.primitive_poses.append(p); co.operation = CollisionObject.ADD
    return co
def apply_scene(r):
    cli = r.node.create_client(ApplyPlanningScene, "/apply_planning_scene"); cli.wait_for_service(10)
    req = ApplyPlanningScene.Request(); req.scene.is_diff = True
    req.scene.world.collision_objects = [
        box("table", (-0.1, 0.0, 0.85), (1.6, 1.6, 0.10)),          # top at 0.90
        box("cabinet", (-0.11, -0.33, 1.014), (0.26, 0.20, 0.228)),  # x[-0.24,0.02] y[-0.43,-0.23] top 1.128
        box("bottle", (0.045, -0.07, 0.98), (0.07, 0.08, 0.16)),
        box("shelf", (0.0, 0.30, 1.07), (0.30, 0.30, 0.34)),   # wooden rack x[-0.15,0.15] y[0.15,0.45] top 1.24
    ]
    fut = cli.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    return fut.result().success
def validator(r):
    cli = r.node.create_client(GetStateValidity, "/check_state_validity"); cli.wait_for_service(10)
    def valid(q):
        req = GetStateValidity.Request(); req.group_name = M["planning"]["group"]
        req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = [float(v) for v in q]
        fut = cli.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
        res = fut.result()
        return res.valid, [(c.contact_body_1, c.contact_body_2) for c in res.contacts]
    return valid
if __name__ == "__main__":
    r = Robot("scene"); print("apply", apply_scene(r))
    valid = validator(r)
    print("current valid", valid(r.arm_q()))
    print("qb (collided) valid", valid(list(np.load("qb.npy"))))
    print("mid collided", valid([0.084,0.415,0.429,-2.06,-1.24,1.081,-2.55]))
