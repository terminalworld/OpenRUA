"""Publish the measured scene (cabinet, drawer, table) as MoveIt collision
boxes and check joint configurations for collisions."""
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from shape_msgs.msg import SolidPrimitive

from arm import JOINTS


def box(name, lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    co = CollisionObject()
    co.header.frame_id = "world"
    co.id = name
    sp = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(hi - lo))
    p = Pose()
    p.position.x, p.position.y, p.position.z = map(float, (lo + hi) / 2)
    p.orientation.w = 1.0
    co.primitives = [sp]
    co.primitive_poses = [p]
    co.operation = CollisionObject.ADD
    return co


# measured world-frame boxes (see depth analysis)
BOXES = {
    "table": ([-0.7, -0.9, 0.80], [0.7, 0.9, 0.90]),
    "cabinet": ([-0.24, -0.46, 0.90], [0.03, -0.22, 1.127]),
    "top_handle": ([-0.15, -0.22, 1.08], [-0.06, -0.19, 1.11]),
    "drawer_floor": ([-0.215, -0.22, 0.90], [0.01, -0.065, 0.924]),
    "drawer_wall_xneg": ([-0.215, -0.22, 0.924], [-0.205, -0.09, 0.983]),
    "drawer_wall_xpos": ([0.0, -0.22, 0.924], [0.01, -0.09, 0.983]),
    "drawer_front": ([-0.215, -0.09, 0.924], [0.01, -0.065, 0.984]),
    "drawer_handle": ([-0.14, -0.065, 0.945], [-0.10, -0.04, 0.96]),
}


class Scene:
    def __init__(self, arm):
        self.arm = arm
        self.node = arm.node
        self.apply = self.node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.valid = self.node.create_client(GetStateValidity, "/check_state_validity")
        self.apply.wait_for_service(10); self.valid.wait_for_service(10)

    def publish(self, boxes=BOXES):
        ps = PlanningScene(); ps.is_diff = True
        for name, (lo, hi) in boxes.items():
            ps.world.collision_objects.append(box(name, lo, hi))
        fut = self.apply.call_async(ApplyPlanningScene.Request(scene=ps))
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        return fut.result().success

    def check(self, q, fingers=None):
        req = GetStateValidity.Request()
        req.group_name = "panda_arm"
        names = list(JOINTS); pos = [float(v) for v in q]
        if fingers is not None:
            names += ["panda_finger_joint1", "panda_finger_joint2"]
            pos += [float(fingers), float(fingers)]
        req.robot_state.joint_state.name = names
        req.robot_state.joint_state.position = pos
        fut = self.valid.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None:
            return None, ["timeout"]
        contacts = [f"{c.contact_body_1}<->{c.contact_body_2}" for c in res.contacts]
        return res.valid, contacts


def remove(scene, names):
    ps = PlanningScene(); ps.is_diff = True
    for n in names:
        co = CollisionObject(); co.header.frame_id = "world"; co.id = n
        co.operation = CollisionObject.REMOVE
        ps.world.collision_objects.append(co)
    fut = scene.apply.call_async(ApplyPlanningScene.Request(scene=ps))
    rclpy.spin_until_future_complete(scene.node, fut, timeout_sec=30)
    return fut.result().success
