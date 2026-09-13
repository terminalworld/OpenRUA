import numpy as np
"""Populate the MoveIt planning scene with measured obstacles (world frame)."""
import sys, numpy as np, rclpy
from scipy.spatial.transform import Rotation as Rot
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

def box(name, center, size, yaw=0.0):
    co = CollisionObject(); co.header.frame_id = "world"; co.id = name
    sp = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(s) for s in size])
    p = Pose(); p.position.x, p.position.y, p.position.z = map(float, center)
    q = Rot.from_euler("z", yaw).as_quat(); p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
    co.primitives = [sp]; co.primitive_poses = [p]; co.operation = CollisionObject.ADD
    return co

def cyl(name, center, radius, height, axis=(0, 0, 1)):
    co = CollisionObject(); co.header.frame_id = "world"; co.id = name
    sp = SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[float(height), float(radius)])
    p = Pose(); p.position.x, p.position.y, p.position.z = map(float, center)
    z = np.asarray(axis, float) / np.linalg.norm(axis)
    x = np.cross([0, 1, 0] if abs(z[1]) < 0.9 else [1, 0, 0], z); x /= np.linalg.norm(x)
    q = Rot.from_matrix(np.column_stack([x, np.cross(z, x), z])).as_quat()
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
    co.primitives = [sp]; co.primitive_poses = [p]; co.operation = CollisionObject.ADD
    return co

def remove(name):
    co = CollisionObject(); co.header.frame_id = "world"; co.id = name; co.operation = CollisionObject.REMOVE
    return co

WHITE_MUG = dict(center=(-0.081, -0.281), r=0.047, h=0.112)
YELLOW_MUG = dict(center=(-0.022, 0.027), r=0.050, h=0.106)

def objects(include_white=True):
    objs = [
        box("table", (0.0, 0.0, 0.85), (1.6, 1.4, 0.10)),                       # top at 0.90
        box("microwave", (0.005, 0.365, 1.0035), (0.33, 0.20, 0.207)),          # x[-0.16,0.17] y[0.266,0.464]
        box("mw_panel", (0.125, 0.255, 1.0035), (0.09, 0.03, 0.207)),           # control panel lip
        box("mw_door", (-0.212, 0.18, 1.0035), (0.26, 0.03, 0.207), yaw=np.radians(68.2)),
        cyl("yellow_mug", (*YELLOW_MUG["center"], 0.90 + YELLOW_MUG["h"]/2), YELLOW_MUG["r"], YELLOW_MUG["h"]),
    ]
    if include_white:
        objs.append(cyl("white_mug", (*WHITE_MUG["center"], 0.90 + WHITE_MUG["h"]/2), WHITE_MUG["r"], WHITE_MUG["h"]))
    return objs

def apply(node, cos, attached=None, detach=None):
    cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene"); cli.wait_for_service(timeout_sec=10)
    ps = PlanningScene(); ps.is_diff = True; ps.robot_state.is_diff = True
    ps.world.collision_objects = cos
    if attached is not None: ps.robot_state.attached_collision_objects = [attached]
    if detach is not None:
        a = AttachedCollisionObject(); a.link_name = "panda_hand"; a.object.id = detach; a.object.operation = CollisionObject.REMOVE
        ps.robot_state.attached_collision_objects = [a]
    req = ApplyPlanningScene.Request(scene=ps)
    f = cli.call_async(req); rclpy.spin_until_future_complete(node, f, timeout_sec=30)
    return f.result().success if f.result() else None

if __name__ == "__main__":
    rclpy.init(); node = rclpy.create_node("scene")
    print("applied:", apply(node, objects(include_white="--no-white" not in sys.argv)))
