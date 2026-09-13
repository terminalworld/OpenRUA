#!/usr/bin/env python3
"""Publish/maintain planning-scene collision objects for this scene.
Geometry measured from birdview depth (world frame, table top z=0.900).

  python3 scene.py setup            # table, microwave body, control panel, door, gray mug, yellow mug
  python3 scene.py door <deg>       # re-pose door at opening angle (0 = closed, 118 = as found)
  python3 scene.py remove <id>      # remove an object
  python3 scene.py attach_mug       # attach yellow mug cylinder to the hand (after grasp)
  python3 scene.py detach_mug x y   # detach and re-place standing at x,y
"""
import sys, math
import rclpy
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

TABLE_Z = 0.900
HINGE = (-0.265, -0.325)  # door hinge (vertical axis), world xy
DOOR_LEN = 0.26
DOOR_T = 0.07
MW_TOP = 1.107
MARGIN = 0.03


def box(id_, cx, cy, cz, sx, sy, sz, yaw=0.0, op=CollisionObject.ADD):
    co = CollisionObject()
    co.header.frame_id = "world"
    co.id = id_
    sp = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[sx, sy, sz])
    p = Pose()
    p.position.x, p.position.y, p.position.z = cx, cy, cz
    p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    co.primitives = [sp]
    co.primitive_poses = [p]
    co.operation = op
    return co


def cyl(id_, cx, cy, cz, h, r, op=CollisionObject.ADD):
    co = CollisionObject()
    co.header.frame_id = "world"
    co.id = id_
    sp = SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[h, r])
    p = Pose()
    p.position.x, p.position.y, p.position.z = cx, cy, cz
    p.orientation.w = 1.0
    co.primitives = [sp]
    co.primitive_poses = [p]
    co.operation = op
    return co


def door(deg):
    """Door panel as a box hinged at HINGE. deg=0 closed (lies along +x
    face y=-0.325), positive = swung open toward -x/-y (as found: ~118)."""
    ang = -math.radians(deg)  # closed direction +x, opens by rotating about -z
    dx, dy = math.cos(ang), math.sin(ang)
    cx = HINGE[0] + dx * DOOR_LEN / 2 - dy * (-DOOR_T / 2)
    cy = HINGE[1] + dy * DOOR_LEN / 2 + dx * (-DOOR_T / 2)
    return box("mw_door", cx, cy, (TABLE_Z + MW_TOP + MARGIN) / 2, DOOR_LEN + 0.04, DOOR_T,
               MW_TOP + MARGIN - TABLE_Z, yaw=ang)


def apply(objs, attached=None):
    rclpy.init()
    n = rclpy.create_node("scene_setup")
    cli = n.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not cli.wait_for_service(timeout_sec=10):
        raise SystemExit("no apply_planning_scene")
    ps = PlanningScene()
    ps.is_diff = True
    ps.world.collision_objects = objs
    if attached:
        ps.robot_state.attached_collision_objects = attached
        ps.robot_state.is_diff = True
    fut = cli.call_async(ApplyPlanningScene.Request(scene=ps))
    rclpy.spin_until_future_complete(n, fut, timeout_sec=30)
    print("applied:", fut.result().success if fut.result() else "timeout")
    rclpy.shutdown()


def main():
    cmd = sys.argv[1]
    if cmd == "setup":
        objs = [
            box("table", -0.015, 0.0, TABLE_Z - 0.05, 0.95, 1.10, 0.10),
            box("mw_body", -0.095, -0.225, (TABLE_Z + MW_TOP + MARGIN) / 2, 0.34 + 0.02, 0.20, MW_TOP + MARGIN - TABLE_Z),
            box("mw_panel", 0.035, -0.335, (TABLE_Z + MW_TOP) / 2, 0.08, 0.02, MW_TOP - TABLE_Z),
            door(118.0),
            cyl("gray_mug", -0.001, 0.343, TABLE_Z + 0.09, 0.18, 0.055),
            cyl("yellow_mug", -0.012, 0.026, TABLE_Z + 0.09, 0.18, 0.055),
        ]
        apply(objs)
    elif cmd == "cavity":
        # replace solid body with hollow walls: interior x -0.24..-0.03, y -0.325..-0.16, z 0.94..1.095
        rm = CollisionObject(); rm.id = "mw_body"; rm.header.frame_id = "world"; rm.operation = CollisionObject.REMOVE
        x0, x1, y0, y1 = -0.265, 0.075, -0.325, -0.125
        objs = [rm,
            box("mw_floor", (x0+x1)/2, (y0+y1)/2, (TABLE_Z+0.94)/2, x1-x0, y1-y0, 0.94-TABLE_Z),
            box("mw_top", (x0+x1)/2, (y0+y1)/2, (1.095+MW_TOP+MARGIN)/2, x1-x0, y1-y0, MW_TOP+MARGIN-1.095),
            box("mw_back", (x0+x1)/2, (-0.16+y1)/2, (TABLE_Z+MW_TOP)/2, x1-x0, y1+0.16, MW_TOP-TABLE_Z),
            box("mw_left", (x0-0.24)/2, (y0+y1)/2, (TABLE_Z+MW_TOP)/2, -0.24-x0, y1-y0, MW_TOP-TABLE_Z),
            box("mw_right", (-0.03+x1)/2, (y0+y1)/2, (TABLE_Z+MW_TOP)/2, x1+0.03, y1-y0, MW_TOP-TABLE_Z),
        ]
        apply(objs)
    elif cmd == "door":
        apply([door(float(sys.argv[2]))])
    elif cmd == "remove":
        co = CollisionObject(); co.id = sys.argv[2]; co.header.frame_id = "world"
        co.operation = CollisionObject.REMOVE
        apply([co])
    elif cmd == "attach_mug":
        # remove world object, attach to hand as a cylinder hanging below fingertips
        rm = CollisionObject(); rm.id = "yellow_mug"; rm.header.frame_id = "world"
        rm.operation = CollisionObject.REMOVE
        aco = AttachedCollisionObject()
        aco.link_name = "panda_hand"
        aco.touch_links = ["panda_hand", "panda_leftfinger", "panda_rightfinger"]
        co = CollisionObject(); co.header.frame_id = "panda_hand"; co.id = "yellow_mug"
        # approximate: mug axis along hand z, body centred 0.05 along hand -y? keep generic:
        # a 0.11 radius, 0.18 tall cylinder whose top is at fingertip level
        sp = SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[0.18, 0.055])
        p = Pose(); p.position.z = 0.1034 - 0.02 + 0.09; p.position.y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
        p.orientation.w = 1.0
        co.primitives = [sp]; co.primitive_poses = [p]; co.operation = CollisionObject.ADD
        aco.object = co
        apply([rm], attached=[aco])
    elif cmd == "detach_mug":
        aco = AttachedCollisionObject(); aco.link_name = "panda_hand"
        aco.object.id = "yellow_mug"; aco.object.operation = CollisionObject.REMOVE
        x, y = float(sys.argv[2]), float(sys.argv[3])
        apply([cyl("yellow_mug", x, y, TABLE_Z + 0.09, 0.18, 0.055)], attached=[aco])


if __name__ == "__main__":
    main()
