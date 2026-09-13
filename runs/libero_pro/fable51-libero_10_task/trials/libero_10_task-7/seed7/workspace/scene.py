"""Publish table + basket as collision boxes into the MoveIt planning scene."""
import rclpy, time
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

def box(name, center, size):
    co = CollisionObject(); co.id = name; co.header.frame_id = "world"
    sp = SolidPrimitive(); sp.type = SolidPrimitive.BOX; sp.dimensions = [float(s) for s in size]
    p = Pose(); p.position.x, p.position.y, p.position.z = map(float, center); p.orientation.w = 1.0
    co.primitives = [sp]; co.primitive_poses = [p]; co.operation = CollisionObject.ADD
    return co

rclpy.init(); node = rclpy.create_node("scene")
cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
assert cli.wait_for_service(20)
ps = PlanningScene(); ps.is_diff = True
ps.world.collision_objects = [
    box("table", (-0.245, 0.0, 0.375), (1.15, 1.8, 0.10)),   # top at z=0.425
    box("basket", (0.01, 0.26, 0.53), (0.17, 0.18, 0.21)),   # rim at z~0.635
]
req = ApplyPlanningScene.Request(); req.scene = ps
fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
print("applied:", fut.result().success)
g = node.create_client(GetPlanningScene, "/get_planning_scene"); g.wait_for_service(10)
from moveit_msgs.msg import PlanningSceneComponents
rq = GetPlanningScene.Request(); rq.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
fut = g.call_async(rq); rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
print("scene objects:", [c.id for c in fut.result().scene.world.collision_objects])
