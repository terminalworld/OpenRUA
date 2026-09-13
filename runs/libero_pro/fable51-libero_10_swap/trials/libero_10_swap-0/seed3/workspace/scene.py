"""Publish basket + table collision boxes into the MoveIt planning scene."""
from rob import *
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

def box(name, center, size):
    co = CollisionObject()
    co.header.frame_id = "world"
    co.id = name
    sp = SolidPrimitive(); sp.type = SolidPrimitive.BOX; sp.dimensions = [float(s) for s in size]
    p = Pose(); p.position.x, p.position.y, p.position.z = map(float, center); p.orientation.w = 1.0
    co.primitives = [sp]; co.primitive_poses = [p]
    co.operation = CollisionObject.ADD
    return co

def publish_scene(r):
    cli = r.node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    assert cli.wait_for_service(timeout_sec=20)
    req = ApplyPlanningScene.Request()
    req.scene = PlanningScene(); req.scene.is_diff = True
    req.scene.world.collision_objects = [
        box("basket", (0.003, 0.251, 0.525), (0.17, 0.18, 0.20)),
        box("table", (0.0, 0.0, 0.40), (1.6, 1.2, 0.05)),
    ]
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    log("apply_planning_scene ->", fut.result().success if fut.result() else None)

if __name__ == "__main__":
    r = Robot(); publish_scene(r)
