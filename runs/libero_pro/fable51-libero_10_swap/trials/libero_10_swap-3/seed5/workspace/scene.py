#!/usr/bin/env python3
"""Publish measured scene geometry as MoveIt collision objects (world frame),
and offer a state-validity check helper."""
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

TABLE = 0.9


def box(name, xr, yr, zr):
    co = CollisionObject()
    co.header.frame_id = "world"
    co.id = name
    co.operation = CollisionObject.ADD
    sp = SolidPrimitive(type=SolidPrimitive.BOX,
                        dimensions=[float(xr[1] - xr[0]), float(yr[1] - yr[0]), float(zr[1] - zr[0])])
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(np.mean(xr)), float(np.mean(yr)), float(np.mean(zr)))
    p.orientation.w = 1.0
    co.primitives.append(sp)
    co.primitive_poses.append(p)
    return co


def cyl(name, x, y, r, zr):
    co = CollisionObject()
    co.header.frame_id = "world"
    co.id = name
    co.operation = CollisionObject.ADD
    sp = SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[float(zr[1] - zr[0]), float(r)])
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(np.mean(zr))
    p.orientation.w = 1.0
    co.primitives.append(sp)
    co.primitive_poses.append(p)
    return co


def objects(drawer_front=-0.085, bowl=(-0.138, 0.063), with_bowl=True):
    """drawer_front: y of the front panel's inner face."""
    T = TABLE
    objs = [
        box("table", (-0.6, 0.7), (-0.7, 0.7), (T - 0.05, T)),
        box("cabinet", (-0.245, 0.025), (-0.42, -0.225), (T, T + 0.23)),
        box("top_handle", (-0.155, -0.055), (-0.225, -0.19), (T + 0.175, T + 0.205)),
        box("drawer_floor", (-0.225, 0.005), (-0.225, drawer_front), (T, T + 0.025)),
        box("drawer_wall_l", (-0.225, -0.205), (-0.225, drawer_front), (T, T + 0.08)),
        box("drawer_wall_r", (-0.015, 0.005), (-0.225, drawer_front), (T, T + 0.08)),
        box("drawer_front", (-0.225, 0.005), (drawer_front, drawer_front + 0.015), (T, T + 0.09)),
        box("drawer_handle", (-0.155, -0.065), (drawer_front + 0.015, drawer_front + 0.055), (T + 0.04, T + 0.065)),
        cyl("bottle", 0.04, -0.03, 0.035, (T, T + 0.165)),
    ]
    if with_bowl:
        objs.append(cyl("bowl", bowl[0], bowl[1], 0.056, (T, T + 0.053)))
    return objs


class Scene:
    def __init__(self, node):
        self.node = node
        self.apply = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.valid = node.create_client(GetStateValidity, "/check_state_validity")
        self.apply.wait_for_service(timeout_sec=20)
        self.valid.wait_for_service(timeout_sec=20)

    def publish(self, objs, remove=()):
        ps = PlanningScene()
        ps.is_diff = True
        for o in objs:
            ps.world.collision_objects.append(o)
        for name in remove:
            co = CollisionObject(); co.id = name; co.header.frame_id = "world"
            co.operation = CollisionObject.REMOVE
            ps.world.collision_objects.append(co)
        req = ApplyPlanningScene.Request(scene=ps)
        fut = self.apply.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        return fut.result().success if fut.result() else None

    def check(self, arm_names, q, finger=0.04):
        req = GetStateValidity.Request()
        js = JointState()
        js.name = list(arm_names) + ["panda_finger_joint1", "panda_finger_joint2"]
        js.position = [float(v) for v in q] + [float(finger), float(finger)]
        req.robot_state.joint_state = js
        req.group_name = "panda_arm"
        fut = self.valid.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None:
            return None, ["no answer"]
        contacts = [f"{c.contact_body_1}<->{c.contact_body_2} depth={c.depth:.3f}" for c in res.contacts]
        return res.valid, contacts
